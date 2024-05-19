/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "front_end/gem/lineplane_extractor.h"
#include "front_end/graph_vertex.h"
#include "robot_utils/tic_toc.h"
#include "front_end/gem/clustering.h"
#include <pcl/io/pcd_io.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace g3reg {

    void PLCExtractor::ExtractFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz,
                                      std::vector<LineFeature::Ptr> &line_features,
                                      std::vector<SurfaceFeature::Ptr> &surface_features,
                                      std::vector<ClusterFeature::Ptr> &cluster_features) {
        reset();

        robot_utils::TicToc t;
        double tSrc, ground_time, plane_time, cluster_time, line_time;
        pcl::PointCloud<pcl::PointXYZ> cloud_ground;
        pcl::PointCloud<pcl::PointXYZ> cloud_nonground;
        travel::estimateGround(*cloud_xyz, cloud_ground, cloud_nonground, tSrc);
        ground_time = t.toc();

        cutCloud(cloud_nonground, FeatureType::None, config::voxel_resolution, voxel_map);
        if (config::plane_aided) {
            for (auto voxel_iter = voxel_map.begin(); voxel_iter != voxel_map.end(); ++voxel_iter) {
                Voxel::Ptr voxel = voxel_iter->second;
                if (voxel->parse()) {
                    if (config::plane_aided) {
                        voxel->setSemanticType(FeatureType::Plane);
                    }
                }
            }
            MergePlanes(surface_features);
            FilterSurface(surface_features, config::min_cluster_size);
            plane_time = t.toc();
        } else {
            for (auto voxel_iter = voxel_map.begin(); voxel_iter != voxel_map.end(); ++voxel_iter) {
                voxel_iter->second->solveCenter();
            }
        }

        auto voxel_iter = voxel_map.begin();
        pcl::PointCloud<pcl::PointXYZ>::Ptr other_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (; voxel_iter != voxel_map.end(); ++voxel_iter) {
            if (voxel_iter->second->type() != FeatureType::Plane) {
                *other_cloud += *(voxel_iter->second->cloud());
                voxel_iter->second->setSemanticType(FeatureType::Cluster);
            }
        }

        if (config::cluster_mtd == "travel") {
            travel::Cluster(other_cloud, cluster_features, false);
        } else if (config::cluster_mtd == "dcvc") {
            DCVC::Cluster(other_cloud, cluster_features, false);
        }

        cluster_time = t.toc();

        if (config::num_lines > 0) {
            ExtractPole(cluster_features, line_features);
            for (int i = 0; i < line_features.size(); ++i) {
                LineFeature::Ptr line_feature = line_features[i];
                for (int j = 0; j < line_feature->cloud()->size(); ++j) {
                    VoxelKey key = point_to_voxel_key(line_feature->cloud()->points[j], config::voxel_resolution);
                    auto voxel_iter = voxel_map.find(key);
                    if (voxel_iter != voxel_map.end()) {
                        voxel_iter->second->setSemanticType(FeatureType::Line);
                        voxel_iter->second->setDirection(line_feature->direction());
                    }
                }
            }
            line_time = t.toc();
        }

        if (!config::plane_aided) {
            ExtractPlanes(cluster_features, surface_features);
            for (SurfaceFeature::Ptr surface_feature: surface_features) {
                for (int j = 0; j < surface_feature->cloud()->size(); ++j) {
                    VoxelKey key = point_to_voxel_key(surface_feature->cloud()->points[j], config::voxel_resolution);
                    auto voxel_iter = voxel_map.find(key);
                    if (voxel_iter != voxel_map.end()) {
                        voxel_iter->second->setSemanticType(FeatureType::Plane);
                        voxel_iter->second->setNormal(surface_feature->normal());
                    }
                }
            }
            plane_time = t.toc();
        }

//    LOG(INFO) << "ExtractFeature time: ground/planar/cluster/line: " << ground_time << "/" << plane_time << "/" << cluster_time << "/" << line_time;
    }

    void PLCExtractor::ExtractPlanes(std::vector<ClusterFeature::Ptr> &cluster_features,
                                     std::vector<SurfaceFeature::Ptr> &plane_features) {
        plane_features.clear();
        std::vector<int> remove_index;
        for (int i = 0; i < cluster_features.size(); ++i) {

            ClusterFeature::Ptr cluster_feature = cluster_features[i];
            const Eigen::Vector3d &eigen_values = cluster_feature->eigen_values();

            if (eigen_values(1) / eigen_values(0) < config::eigenvalue_thresh) {
                continue;
            }

            SurfaceFeature::Ptr feature = SurfaceFeature::Ptr(new SurfaceFeature(cluster_feature->cloud()));
            remove_index.push_back(i);
            plane_features.emplace_back(feature);
        }

        // remove clusters from cluster_features
        for (int i = remove_index.size() - 1; i >= 0; --i) {
            cluster_features.erase(cluster_features.begin() + remove_index[i]);
        }
    }

    void PLCExtractor::ExtractPole(std::vector<ClusterFeature::Ptr> &cluster_features,
                                   std::vector<LineFeature::Ptr> &line_features) {
        line_features.clear();
        std::vector<int> remove_index;
        for (int i = 0; i < cluster_features.size(); ++i) {

            ClusterFeature::Ptr cluster_feature = cluster_features[i];
            Eigen::Vector3d eigen_values = cluster_feature->eigen_values();
            if (eigen_values[2] / eigen_values[0] < config::eigenvalue_thresh / 2.0) {
                continue;
            }
            double angle = abs(cluster_feature->direction().dot(Eigen::Vector3d(0, 0, 1)));
            if (angle < 0.707) {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = cluster_feature->cloud();
            // using RANSAC 拟合直线，判断内点率和内点个数是否满足条件
            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(
                    new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cluster_cloud)
            );
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
            ransac.setDistanceThreshold(0.5);
            ransac.setMaxIterations(10);
            ransac.computeModel();
            std::vector<int> inliers;
            ransac.getInliers(inliers);
            double inlier_ratio = (double) inliers.size() / cluster_cloud->size();
            if (inlier_ratio < 0.5 || inliers.size() < 5) {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ> pole_line;
            pcl::copyPointCloud<pcl::PointXYZ>(*cluster_cloud, inliers, pole_line);
            LineFeature::Ptr feature = LineFeature::Ptr(new LineFeature());
            if (feature->Init(pole_line.makeShared())) {
                remove_index.push_back(i);
                line_features.emplace_back(feature);
            }
        }

        // remove clusters from cluster_features
        for (int i = remove_index.size() - 1; i >= 0; --i) {
            cluster_features.erase(cluster_features.begin() + remove_index[i]);
        }
    }

    void
    PLCExtractor::MergePlanes(std::vector<SurfaceFeature::Ptr> &surface_features) {

        //    merge surface
        std::map<int, SurfaceFeature::Ptr> surface_map;
        int global_id = 0;
        for (auto voxel_iter = voxel_map.begin(); voxel_iter != voxel_map.end(); ++voxel_iter) {
            const VoxelKey &loc = voxel_iter->first;
            Voxel::Ptr cur_voxel = voxel_iter->second;
            if (cur_voxel->type() != FeatureType::Plane) continue;

            if (cur_voxel->instance_id < 0) {
                SurfaceFeature::Ptr surface(
                        new SurfaceFeature(cur_voxel->center(), cur_voxel->normal(), cur_voxel->sigma()));
                surface->setCloud(cur_voxel->cloud());
                surface->push_back(loc);
                surface_map[global_id] = surface;
                cur_voxel->instance_id = global_id;
                global_id++;
            }
            SurfaceFeature::Ptr surface = surface_map[cur_voxel->instance_id];
            if (!surface) {
                LOG(INFO) << "surface is null";
            }
            std::vector<VoxelKey> neighbors;
            cur_voxel->getNeighbors(neighbors);
            for (VoxelKey &neighbor: neighbors) {
                auto neighbor_iter = voxel_map.find(neighbor);
                // if neighbor is not in voxel_map
                if (neighbor_iter == voxel_map.end()) continue;
                Voxel &neighbor_voxel = *neighbor_iter->second;
                // if neighbor is not a surface voxel, continue
                if (neighbor_voxel.type() == FeatureType::None) continue;
                // if neighbor has not been assigned to a surface
                if (neighbor_voxel.instance_id < 0) {
                    if (surface->consistent(neighbor_voxel)) {
                        surface->merge(neighbor_voxel);
                        neighbor_voxel.instance_id = cur_voxel->instance_id;
                    }
                } else {
                    // if neighbor has been assigned to a surface, try to merge
                    if (neighbor_voxel.instance_id == cur_voxel->instance_id) continue;
                    SurfaceFeature::Ptr neighbor_surface = surface_map[neighbor_voxel.instance_id];
                    if (surface->consistent(*neighbor_surface)) {
                        surface->merge(*neighbor_surface);
                        surface_map.erase(neighbor_voxel.instance_id);
                        for (auto &voxel_loc: neighbor_surface->voxels()) {
                            voxel_map[voxel_loc]->instance_id = cur_voxel->instance_id;
                        }
                    }
                }
            }
        }

        surface_features.clear();
        for (auto &surface_pair: surface_map) {
            surface_features.emplace_back(surface_pair.second);
        }
    }

    void PLCExtractor::FilterSurface(std::vector<SurfaceFeature::Ptr> &surface_features, int min_points) {

        // only save the surfaces with more than min_points
        std::vector<SurfaceFeature::Ptr> filtered_surface_features;
        for (auto &surface_feature: surface_features) {
            double angle = abs(surface_feature->normal().dot(Eigen::Vector3d(0, 0, 1)));
            if (surface_feature->cloud()->size() > min_points && angle < 0.707) {
                filtered_surface_features.emplace_back(surface_feature);
            } else {
                for (VoxelKey &loc: surface_feature->voxels()) {
                    voxel_map[loc]->setSemanticType(FeatureType::None);
                }
            }
        }
        surface_features = filtered_surface_features;
    }

    void PLCExtractor::reset() {
        voxel_map.clear();
    }

    void TopKEllipse(std::vector<g3reg::QuadricFeature::Ptr> &ellipsoids, int k) {
        ellipsoids.erase(std::remove_if(ellipsoids.begin(), ellipsoids.end(), [](g3reg::QuadricFeature::Ptr ellipsoid) {
            const double &norm = ellipsoid->center().norm();
            return (norm > config::max_range || norm < config::min_range);
        }), ellipsoids.end());
        if (ellipsoids.size() < k)
            return;
        std::vector<std::pair<double, g3reg::QuadricFeature::Ptr>> ellipsoids_score;
        for (auto &ellipsoid: ellipsoids) {
            ellipsoids_score.emplace_back(std::make_pair(ellipsoid->score(), ellipsoid));
        }
        std::sort(ellipsoids_score.begin(), ellipsoids_score.end(),
                  [](std::pair<double, g3reg::QuadricFeature::Ptr> p1,
                     std::pair<double, g3reg::QuadricFeature::Ptr> p2) {
                      return p1.first > p2.first;
                  });
        ellipsoids.clear();
        for (int i = 0; i < k; i++) {
            ellipsoids.push_back(ellipsoids_score[i].second);
        }
    }

    void TransformToEllipsoid(const FeatureSet &featureSet, std::vector<std::vector<QuadricFeature::Ptr>> &ellipsoids) {
        ellipsoids.clear();
        std::vector<g3reg::QuadricFeature::Ptr> ellipsoid_lines;
        for (auto &line: featureSet.lines) {
            g3reg::QuadricFeature::Ptr quadric_feature = std::dynamic_pointer_cast<g3reg::QuadricFeature>(line);
            ellipsoid_lines.push_back(quadric_feature);
        }
        TopKEllipse(ellipsoid_lines, config::num_lines);
        ellipsoids.push_back(ellipsoid_lines);

        std::vector<g3reg::QuadricFeature::Ptr> ellipsoid_planes;
        for (auto &plane: featureSet.planes) {
            g3reg::QuadricFeature::Ptr quadric_feature = std::dynamic_pointer_cast<g3reg::QuadricFeature>(plane);
            ellipsoid_planes.push_back(quadric_feature);
        }
        TopKEllipse(ellipsoid_planes, config::num_planes);
        ellipsoids.push_back(ellipsoid_planes);

        std::vector<g3reg::QuadricFeature::Ptr> ellipsoid_clusters;
        for (auto &cluster: featureSet.clusters) {
            g3reg::QuadricFeature::Ptr quadric_feature = std::dynamic_pointer_cast<g3reg::QuadricFeature>(cluster);
            ellipsoid_clusters.push_back(quadric_feature);
        }
        TopKEllipse(ellipsoid_clusters, config::num_clusters);
        ellipsoids.push_back(ellipsoid_clusters);

        if (config::use_pseudo_cov) {
            for (int i = 0; i < ellipsoids.size(); ++i) {
                for (int j = 0; j < ellipsoids[i].size(); ++j) {
                    ellipsoids[i][j]->fitting();
                }
            }
        }
//        LOG(INFO) << "TransformToEllipsoid: " << ellipsoids[0].size() << " lines, " << ellipsoids[1].size() << " planes, " << ellipsoids[2].size() << " clusters" << std::endl;
    }
}