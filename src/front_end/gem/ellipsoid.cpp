/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "front_end/gem/ellipsoid.h"
#include <thread>
#include "utils/config.h"
#include "robot_utils/algorithms.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include "front_end/gem/gem_matching.h"

namespace g3reg{

    std::vector<std::vector<Eigen::VectorXd>> computeFPFHFeatures(std::vector<std::vector<g3reg::QuadricFeature::Ptr>>& ellipsoids,
                                                                   double neighborhood_radius = 20.0) {
        // Intermediate variables
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

        // Estimate normals
        for (int i = 0; i < ellipsoids.size(); ++i) {
            for (int j = 0; j < ellipsoids[i].size(); ++j) {
                FeatureType type = ellipsoids[i][j]->type();
                Eigen::Vector3d center = ellipsoids[i][j]->center();
                input_cloud->push_back(pcl::PointXYZ(center(0), center(1), center(2)));
                if (type==FeatureType::Plane){
                    Eigen::Vector3d normal = ellipsoids[i][j]->normal();
                    normals->push_back(pcl::Normal(normal(0), normal(1), normal(2)));
                } else if (type==FeatureType::Line){
                    Eigen::Vector3d direction = ellipsoids[i][j]->direction();
                    normals->push_back(pcl::Normal(direction(0), direction(1), direction(2)));
                } else if (type==FeatureType::Cluster){
                    Eigen::Vector3d normal = ellipsoids[i][j]->normal();
                    normals->push_back(pcl::Normal(normal(0), normal(1), normal(2)));
                }
            }
        }

        // Estimate FPFH
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
        fpfh_estimation.setInputCloud(input_cloud);
        fpfh_estimation.setInputNormals(normals);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        fpfh_estimation.setSearchMethod(kdtree);
        fpfh_estimation.setRadiusSearch(neighborhood_radius);
        fpfh_estimation.compute(*descriptors);

        std::vector<std::vector<Eigen::VectorXd>> descriptors_vec(ellipsoids.size());
        int idx = 0;
        int size_descriptor = (*descriptors)[0].descriptorSize();
        for (int i = 0; i < ellipsoids.size(); ++i) {
            descriptors_vec[i].resize(ellipsoids[i].size());
            for (int j = 0; j < ellipsoids[i].size(); ++j) {
                Eigen::VectorXd fpfh(33);
                for (int i = 0; i < size_descriptor; i++)
                    fpfh(i) = descriptors->at(idx).histogram[i];
                descriptors_vec[i][j] = fpfh;
                idx++;
            }
        }

        return descriptors_vec;
    }

    Eigen::Vector3i Desc2Key(const Eigen::Vector3d &point) {
        const Eigen::Vector3d voxel_size = {0.5, 0.5, 1.0};
        int x = static_cast<int>(std::floor(point.x() / voxel_size.x()));
        int y = static_cast<int>(std::floor(point.y() / voxel_size.y()));
        int z = static_cast<int>(std::floor(point.z() / voxel_size.z()));
        return Eigen::Matrix<int, 3, 1>(x, y, z);
    }

    double distancePCL(pcl::PointXYZ& p1, pcl::PointXYZ& p2, Eigen::Vector3d& axis, FeatureType type){
        Eigen::Vector3d delta = (p1.getVector3fMap().cast<double>()-p2.getVector3fMap().cast<double>());
        if (type==FeatureType::Line){
            return ((Eigen::Matrix3d::Identity()-axis*axis.transpose())*delta).norm();
        } else{
            return abs(delta.dot(axis));
        }
    }

// Assuming that PointT is a type that represents a 3D point, e.g., pcl::PointXYZ
    std::vector<DescMap> computeHashDesc(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<Eigen::Vector3d>& normals, std::vector<FeatureType> labels, double neighborhood_radius = 20.0) {
        // Construct the KDTree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        // For each point in the cloud
        std::vector<DescMap> hash_desc(cloud->points.size());
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            std::unordered_map<Eigen::Vector3i, int, robot_utils::hash_vec<3>> voxel_map;
            // Perform radius search
            if (kdtree.radiusSearch(cloud->points[i], neighborhood_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                // For each neighbor
                for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
                    Eigen::Vector3d vec = cloud->points[pointIdxRadiusSearch[j]].getVector3fMap().cast<double>() - cloud->points[i].getVector3fMap().cast<double>();
                    double dist1 = distancePCL(cloud->points[pointIdxRadiusSearch[j]], cloud->points[i], normals[i], labels[i]);
                    double dist2 = distancePCL(cloud->points[pointIdxRadiusSearch[j]], cloud->points[i], normals[pointIdxRadiusSearch[j]], labels[pointIdxRadiusSearch[j]]);
                    double angle = std::acos(normals[i].dot(normals[pointIdxRadiusSearch[j]])) / M_PI * 180.0;

                    Eigen::Vector3i hash_val = Desc2Key(Eigen::Vector3d(std::max(dist1, dist2), std::min(dist1, dist2), angle));
                    auto iter = voxel_map.find(hash_val);
                    if (iter == voxel_map.end()) {
                        voxel_map[hash_val] = 1;
                    } else {
                        voxel_map[hash_val] += 1;
                    }
                }
                // For each neighbor and other neighbor
//                for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
//                    for (size_t k = j + 1; k < pointIdxRadiusSearch.size(); ++k) {
//                        Eigen::Vector3d vec = cloud->points[pointIdxRadiusSearch[j]].getVector3fMap().cast<double>() - cloud->points[pointIdxRadiusSearch[k]].getVector3fMap().cast<double>();
//                        double dist1 = distancePCL(cloud->points[pointIdxRadiusSearch[j]], cloud->points[pointIdxRadiusSearch[k]], normals[pointIdxRadiusSearch[k]], labels[pointIdxRadiusSearch[k]]);
//                        double dist2 = distancePCL(cloud->points[pointIdxRadiusSearch[j]], cloud->points[pointIdxRadiusSearch[k]], normals[pointIdxRadiusSearch[j]], labels[pointIdxRadiusSearch[j]]);
//                        double angle = std::acos(normals[pointIdxRadiusSearch[j]].dot(normals[pointIdxRadiusSearch[k]])) / M_PI * 180.0;
//
//                        Eigen::Vector3i hash_val = Desc2Key(Eigen::Vector3d(std::max(dist1, dist2), std::min(dist1, dist2), angle));
//                        auto iter = voxel_map.find(hash_val);
//                        if (iter == voxel_map.end()) {
//                            voxel_map[hash_val] = 1;
//                        } else {
//                            voxel_map[hash_val] += 1;
//                        }
//                    }
//                }
            }
            hash_desc[i] = voxel_map;
        }

        return hash_desc;
    }

    void EllipsoidMatcher::extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud) {
        
        robot_utils::TicToc extract_timer;
        std::thread src_thread([&src_cloud, this](){
            plc_src_.ExtractFeature(src_cloud, src_features_.lines, src_features_.planes, src_features_.clusters);
        });

        std::thread tgt_thread([&tgt_cloud, this](){
            plc_tgt_.ExtractFeature(tgt_cloud, tgt_features_.lines, tgt_features_.planes, tgt_features_.clusters);
        });
        src_thread.join();
        tgt_thread.join();

        //LOG(INFO) << "Extract Ellipsoid Time: " << extract_timer.toc() << " ms" << std::endl;
        
        TransformToEllipsoid(src_features_, src_ellipsoids_vec_);
        TransformToEllipsoid(tgt_features_, tgt_ellipsoids_vec_);
    };

    void EllipsoidMatcher::associateAdvanced() {
        int semantic_num = src_ellipsoids_vec_.size();
        for (int i = 0; i < semantic_num; ++i) {
            src_ellipsoids_.insert(src_ellipsoids_.end(), src_ellipsoids_vec_[i].begin(), src_ellipsoids_vec_[i].end());
            tgt_ellipsoids_.insert(tgt_ellipsoids_.end(), tgt_ellipsoids_vec_[i].begin(), tgt_ellipsoids_vec_[i].end());
        }

        std::vector<std::vector<GEM::Ptr>> src_gems_vec(semantic_num), tgt_gems_vec(semantic_num);
        // build GEMs
        double neighborhood_radius = 10.0; // unit: m
        if (config::assoc_method == "fpfh"){
            std::vector<std::vector<Eigen::VectorXd>> src_descriptors = computeFPFHFeatures(src_ellipsoids_vec_, neighborhood_radius);
            std::vector<std::vector<Eigen::VectorXd>> tgt_descriptors = computeFPFHFeatures(tgt_ellipsoids_vec_, neighborhood_radius);
            for (int i = 0; i < semantic_num; ++i) {
                src_gems_vec[i].reserve(src_ellipsoids_vec_[i].size());
                for (int j = 0; j < src_descriptors[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(src_descriptors[i][j]));
                    src_gems_vec[i].push_back(gem);
                }
                for (int j = 0; j < tgt_descriptors[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(tgt_descriptors[i][j]));
                    tgt_gems_vec[i].push_back(gem);
                }
            }
        } else if (config::assoc_method == "hash_desc"){
            pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            std::vector<Eigen::Vector3d> src_normals;
            std::vector<FeatureType> src_labels;
            for (int i = 0; i < src_ellipsoids_vec_.size(); ++i) {
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    pcl::PointXYZ pt;
                    pt.getVector3fMap() = src_ellipsoids_vec_[i][j]->center().cast<float>();
                    src_cloud->push_back(pt);
                    src_normals.push_back(src_ellipsoids_vec_[i][j]->normal());
                    src_labels.push_back(src_ellipsoids_vec_[i][j]->type());
                }
            }
            std::vector<DescMap> src_voxel_map = computeHashDesc(src_cloud, src_normals, src_labels);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            std::vector<Eigen::Vector3d> tgt_normals;
            std::vector<FeatureType> tgt_labels;
            for (int i = 0; i < tgt_ellipsoids_vec_.size(); ++i) {
                for (int j = 0; j < tgt_ellipsoids_vec_[i].size(); ++j) {
                    pcl::PointXYZ pt;
                    pt.getVector3fMap() = tgt_ellipsoids_vec_[i][j]->center().cast<float>();
                    tgt_cloud->push_back(pt);
                    tgt_normals.push_back(tgt_ellipsoids_vec_[i][j]->normal());
                    tgt_labels.push_back(tgt_ellipsoids_vec_[i][j]->type());
                }
            }
            std::vector<DescMap> tgt_voxel_map = computeHashDesc(tgt_cloud, tgt_normals, tgt_labels);

            int index = 0;
            for (int i = 0; i < semantic_num; ++i) {
                src_gems_vec[i].reserve(src_ellipsoids_vec_[i].size());
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM("hash_desc"));
                    gem->setDescMap(src_voxel_map[index]);
                    src_gems_vec[i].push_back(gem);
                    index++;
                }
            }
            index = 0;
            for (int i = 0; i < semantic_num; ++i) {
                tgt_gems_vec[i].reserve(tgt_ellipsoids_vec_[i].size());
                for (int j = 0; j < tgt_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM("hash_desc"));
                    gem->setDescMap(tgt_voxel_map[index]);
                    tgt_gems_vec[i].push_back(gem);
                    index++;
                }
            }
        } else if (config::assoc_method == "wasserstein"){
            for (int i = 0; i < semantic_num; ++i) {
                src_gems_vec[i].reserve(src_ellipsoids_vec_[i].size());
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(src_ellipsoids_vec_[i][j]->eigen_values().cwiseSqrt()));
                    src_gems_vec[i].push_back(gem);
                }
                for (int j = 0; j < tgt_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(tgt_ellipsoids_vec_[i][j]->eigen_values().cwiseSqrt()));
                    tgt_gems_vec[i].push_back(gem);
                }
            }
        } else if (config::assoc_method == "ev_feature"){
            for (int i = 0; i < semantic_num; ++i) {
                src_gems_vec[i].reserve(src_ellipsoids_vec_[i].size());
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(src_ellipsoids_vec_[i][j]->eigenvalue_feature()));
                    src_gems_vec[i].push_back(gem);
                }
                for (int j = 0; j < tgt_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(tgt_ellipsoids_vec_[i][j]->eigenvalue_feature()));
                    tgt_gems_vec[i].push_back(gem);
                }
            }
        } else if (config::assoc_method == "iou3d"){
            for (int i = 0; i < semantic_num; ++i) {
                src_gems_vec[i].reserve(src_ellipsoids_vec_[i].size());
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(src_ellipsoids_vec_[i][j]->size(), "iou3d"));
                    src_gems_vec[i].push_back(gem);
                }
                for (int j = 0; j < tgt_ellipsoids_vec_[i].size(); ++j) {
                    GEM::Ptr gem(new GEM(tgt_ellipsoids_vec_[i][j]->size(), "iou3d"));
                    tgt_gems_vec[i].push_back(gem);
                }
            }
        } else if (config::assoc_method == "all_to_all" || config::assoc_method == "random_select"){

        } else{
            throw std::runtime_error("Unknown association method: " + config::assoc_method);
        }

        // obtain the association
        std::vector<std::pair<int, int>> assoc_vec;
        int src_num = 0, tgt_num = 0;
        if (config::assoc_method == "all_to_all"){
            for (int i = 0; i < semantic_num; ++i) {
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    for (int k = 0; k < tgt_ellipsoids_vec_[i].size(); ++k) {
                        assoc_vec.push_back(std::make_pair(src_num + j, tgt_num + k));
                    }
                }
                src_num += src_ellipsoids_vec_[i].size();
                tgt_num += tgt_ellipsoids_vec_[i].size();
            }
        } else if (config::assoc_method == "random_select"){
            for (int i = 0; i < semantic_num; ++i) {
                int topK = std::min(config::assoc_topk, (int)tgt_ellipsoids_vec_[i].size());
                for (int j = 0; j < src_ellipsoids_vec_[i].size(); ++j) {
                    for (int k = 0; k < topK; ++k) {
                        assoc_vec.push_back(std::make_pair(src_num + j, tgt_num + k));
                    }
                }
                src_num += src_ellipsoids_vec_[i].size();
                tgt_num += topK;
            }
        } else{
            for (int i = 0; i < semantic_num; i++){
                const std::vector<std::pair<int, int>>& assoc_vec_i = MatchingGEMs(src_gems_vec[i], tgt_gems_vec[i], config::assoc_topk);
                for (int j = 0; j < assoc_vec_i.size(); ++j) {
                    assoc_vec.push_back(std::make_pair(src_num + assoc_vec_i[j].first, tgt_num + assoc_vec_i[j].second));
                }
                src_num += src_gems_vec[i].size();
                tgt_num += tgt_gems_vec[i].size();
            }
        }
        A_.resize(assoc_vec.size(), 2);
        for (int i = 0; i < assoc_vec.size(); ++i) {
            A_(i, 0) = assoc_vec[i].first;
            A_(i, 1) = assoc_vec[i].second;
        }
        return;
    }

    clique_solver::Association EllipsoidMatcher::matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                        std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes){
        robot_utils::TicToc tic_toc;
        double extract_time = 0, assoc_time = 0, node_time = 0;
        extractFeatures(src_cloud, tgt_cloud);
        extract_time = tic_toc.toc();

        associateAdvanced();
        assoc_time = tic_toc.toc();

        // construct the matching node pairs
        src_nodes.clear();
        tgt_nodes.clear();
        src_nodes.reserve(src_ellipsoids_.size());
        tgt_nodes.reserve(tgt_ellipsoids_.size());

        for (int i = 0; i < src_ellipsoids_.size(); i++) {
            src_nodes.push_back(src_ellipsoids_[i]->vertex());
        }

        for (int i = 0; i < tgt_ellipsoids_.size(); i++) {
            tgt_nodes.push_back(tgt_ellipsoids_[i]->vertex());
        }
        node_time = tic_toc.toc();
//        LOG(INFO) << "Extract time: " << extract_time << "s, assoc time: " << assoc_time << "s, node time: " << node_time << "ms";

        return A_;
    }

    void EllipsoidMatcher::TopKEllipse(std::vector<g3reg::QuadricFeature::Ptr> &ellipsoids, int k){
        ellipsoids.erase(std::remove_if(ellipsoids.begin(), ellipsoids.end(), [](g3reg::QuadricFeature::Ptr ellipsoid){
            const double& norm = ellipsoid->center().norm();
            return (norm > config::max_range || norm < config::min_range);
        }), ellipsoids.end());
        if (ellipsoids.size() < k)
            return;
        std::vector<std::pair<double, g3reg::QuadricFeature::Ptr>> ellipsoids_score;
        for(auto& ellipsoid: ellipsoids){
            ellipsoids_score.emplace_back(std::make_pair(ellipsoid->score(), ellipsoid));
        }
        std::sort(ellipsoids_score.begin(), ellipsoids_score.end(), [](std::pair<double, g3reg::QuadricFeature::Ptr> p1, std::pair<double, g3reg::QuadricFeature::Ptr> p2){
            return p1.first > p2.first;
        });
        ellipsoids.clear();
        for(int i = 0; i < k; i++){
            ellipsoids.push_back(ellipsoids_score[i].second);
        }
    }

    void EllipsoidMatcher::TransformToEllipsoid(const FeatureSet& featureSet, std::vector<std::vector<QuadricFeature::Ptr>>& ellipsoids){
        ellipsoids.clear();
        std::vector<g3reg::QuadricFeature::Ptr> ellipsoid_lines;
        for(auto& line: featureSet.lines){
            g3reg::QuadricFeature::Ptr quadric_feature = std::dynamic_pointer_cast<g3reg::QuadricFeature>(line);
            ellipsoid_lines.push_back(quadric_feature);
        }
        TopKEllipse(ellipsoid_lines, config::num_lines);
        ellipsoids.push_back(ellipsoid_lines);

        std::vector<g3reg::QuadricFeature::Ptr> ellipsoid_planes;
        for(auto& plane: featureSet.planes){
            g3reg::QuadricFeature::Ptr quadric_feature = std::dynamic_pointer_cast<g3reg::QuadricFeature>(plane);
            ellipsoid_planes.push_back(quadric_feature);
        }
        TopKEllipse(ellipsoid_planes, config::num_planes);
        ellipsoids.push_back(ellipsoid_planes);

        std::vector<g3reg::QuadricFeature::Ptr> ellipsoid_clusters;
        for(auto& cluster: featureSet.clusters){
            g3reg::QuadricFeature::Ptr quadric_feature = std::dynamic_pointer_cast<g3reg::QuadricFeature>(cluster);
            ellipsoid_clusters.push_back(quadric_feature);
        }
        TopKEllipse(ellipsoid_clusters, config::num_clusters);
        ellipsoids.push_back(ellipsoid_clusters);

        if (config::use_pseudo_cov){
            for (int i = 0; i < ellipsoids.size(); ++i) {
                for (int j = 0; j < ellipsoids[i].size(); ++j) {
                    ellipsoids[i][j]->fitting();
                }
            }
        }
//        LOG(INFO) << "TransformToEllipsoid: " << ellipsoids[0].size() << " lines, " << ellipsoids[1].size() << " planes, " << ellipsoids[2].size() << " clusters" << std::endl;
    }

    std::vector<QuadricFeature::Ptr> transformQuadric(const std::vector<QuadricFeature::Ptr>& quadric_vec,
                                                      const Eigen::Matrix4d& T){
        std::vector<QuadricFeature::Ptr> transformed_quadric_vec;
        for (auto& quadric : quadric_vec){
            QuadricFeature::Ptr transformed_quadric = QuadricFeature::Ptr(new QuadricFeature());
            *transformed_quadric = *quadric;
            transformed_quadric->transform(T);
            transformed_quadric_vec.push_back(transformed_quadric);
        }
        return transformed_quadric_vec;
    }


    g3reg::QuadricFeature::Ptr Vertex2QuadricFeature(const clique_solver::GraphVertex::Ptr& node){
        g3reg::QuadricFeature::Ptr feature = g3reg::QuadricFeature::Ptr(new g3reg::QuadricFeature());
        const Eigen::Vector3d& center = node->centroid;
        const Eigen::Matrix3d& covariance = node->covariance;
        // eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
        Eigen::Vector3d eigen_values = eigen_solver.eigenvalues(); // sorted in ascending order
        Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors();
        Eigen::Vector3d size = 2 * config::volume_chi2 * eigen_values.array().sqrt();
        feature->set_center(center);
        feature->set_center_geo(center);
        feature->set_size(size);
        feature->set_rotation(eigen_vectors);
        return feature;
    }

    std::vector<g3reg::QuadricFeature::Ptr> Vertices2QuadricFeatures(const std::vector<clique_solver::GraphVertex::Ptr>& nodes){
        std::vector<g3reg::QuadricFeature::Ptr> features;
        for (auto& node : nodes){
            g3reg::QuadricFeature::Ptr feature = Vertex2QuadricFeature(node);
            features.push_back(feature);
        }
        return features;
    }
}