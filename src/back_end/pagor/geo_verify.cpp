/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "back_end/pagor/geo_verify.h"
#include <pcl/search/kdtree.h>
#include "nanoflann/KDTreeVectorOfVectorsAdaptor.h"
#include "robot_utils/lie_utils.h"

using namespace g3reg;

void mark_free_voxels_by_sampling(const Eigen::Vector3d &start_point, const Eigen::Vector3d &end_point,
                                  VoxelMap &voxel_map) {
    // Initialize the start and end voxel keys
    VoxelKey start_key = point_to_voxel_key(start_point, config.voxel_resolution);
    VoxelKey end_key = point_to_voxel_key(end_point, config.voxel_resolution);

    // Calculate the ray direction
    Eigen::Vector3d ray_direction = (end_point - start_point).normalized();
    double step = config.voxel_resolution[0] / 2.0;
    double total_distance = (end_point - start_point).norm() + config.voxel_resolution[0];
    int num_samples = static_cast<int>(std::floor(total_distance / step));
    for (int i = 0; i < num_samples; ++i) {
        Eigen::Vector3d sample_point = start_point + i * step * ray_direction;
        VoxelKey sample_key = point_to_voxel_key(sample_point, config.voxel_resolution);
        VoxelMap::iterator voxel_iter = voxel_map.find(sample_key);
        if (voxel_iter == voxel_map.end()) {
            Voxel::Ptr voxel = Voxel::Ptr(new Voxel(sample_key, FeatureType::Free));
            voxel_map.insert(std::make_pair(sample_key, voxel));
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AdaptiveDownsample(const VoxelMap &voxel_map, int sample_num) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto voxel: voxel_map) {
        if (sample_num == 1) {
            pcl::PointXYZ pt(voxel.second->center()(0), voxel.second->center()(1), voxel.second->center()(2));
            cloud->push_back(pt);
        } else {
            int pt_size = voxel.second->cloud()->size();
            if (pt_size > sample_num) {
                int step = pt_size / sample_num;
                for (int i = 0; i < sample_num; ++i) {
                    cloud->push_back(voxel.second->cloud()->points[i * step]);
                }
            } else {
                *cloud += *voxel.second->cloud();
            }
        }
    }
    return cloud;
}

float RobustKernel(float residual, float hyper_parameter, std::string type) {
    std::string type_lower = type;
    // transform to lower case
    std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);
    if (type == "huber") {
        if (std::abs(residual) <= hyper_parameter) {
            return 0.5 * residual * residual;
        } else {
            return hyper_parameter * (std::abs(residual) - 0.5 * hyper_parameter);
        }
    } else if (type == "cauchy") {
        return hyper_parameter * hyper_parameter *
               std::log(1.0 + (residual * residual) / (hyper_parameter * hyper_parameter));
    } else if (type == "tukey") {
        if (std::abs(residual) <= hyper_parameter) {
            float temp = 1 - std::pow((residual / hyper_parameter), 2);
            return (hyper_parameter * hyper_parameter / 6.0) * (1 - temp * temp * temp);
        } else {
            return hyper_parameter * hyper_parameter / 6.0;
        }
    } else if (type == "geman_mcclure") {
        // geman-mcclure
        return (residual * residual) / (2.0 * (hyper_parameter * hyper_parameter + residual * residual));
    } else if (type == "tls") {
        // truncated least squares
        return std::min(0.5 * residual * residual, 0.5 * hyper_parameter * hyper_parameter);
    } else if (type == "dcs") {
        // dynamic covariance scaling
        return hyper_parameter * hyper_parameter *
               (1 - std::exp(-residual * residual / (2 * hyper_parameter * hyper_parameter)));
    } else {
        std::cout << "Unknown robust kernel type: " << type << std::endl;
        exit(-1);
    }
}

float ComputeResidual(const Eigen::Vector3f &query, const Eigen::Vector3f &tgt_pt, const VoxelMap &voxel_map_tgt) {
    VoxelKey tgt_key = point_to_voxel_key(tgt_pt, config.voxel_resolution);
    VoxelMap::const_iterator voxel_iter = voxel_map_tgt.find(tgt_key);
    if (voxel_iter == voxel_map_tgt.end()) {
        return (query - tgt_pt).norm();
    }
    if (voxel_iter->second->type() == FeatureType::Plane) {
        const Eigen::Vector3f &normal = voxel_iter->second->normal().cast<float>();
        float residual = (query - tgt_pt).dot(normal);
        return residual;
    } else if (voxel_iter->second->type() == FeatureType::Line) {
        const Eigen::Vector3f &direction = voxel_iter->second->direction().cast<float>();
        float residual = (query - tgt_pt).cross(direction).norm();
        return residual;
    } else {
        return (query - tgt_pt).norm();
    }
}

#include <pcl/io/pcd_io.h>

std::pair<bool, Eigen::Matrix4d> GeometryVerify(const VoxelMap &voxel_map_src, const VoxelMap &voxel_map_tgt,
                                                const std::vector<Eigen::Matrix4d> &candidates) {

    if (candidates.size() == 1) {
        return std::make_pair(true, candidates[0]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    src_cloud = AdaptiveDownsample(voxel_map_src, 5);
    tgt_cloud = AdaptiveDownsample(voxel_map_tgt, 1);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tgt_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tgt_tree->setInputCloud(tgt_cloud);
    float threshold = config.voxel_resolution[0];
    std::pair<float, Eigen::Matrix4d> best_pair = std::make_pair(INFINITY, candidates[0]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4d last_candidate = Eigen::Matrix4d::Identity();
    for (int i = 0; i < candidates.size(); ++i) {
        Eigen::Matrix4d tf = candidates[i];
        if (i > 0 && (tf - last_candidate).norm() < 0.1) continue;
        int intersect_num = 0;
        float total_residual = 0.0, residual;
        pcl::transformPointCloud(*src_cloud, *transformed_src, tf);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        for (int i = 0; i < transformed_src->size(); ++i) {
            tgt_tree->nearestKSearch(transformed_src->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            if (pointIdxNKNSearch[0] < 0 || pointIdxNKNSearch[0] >= tgt_cloud->size()) continue;
            residual = ComputeResidual(transformed_src->points[i].getVector3fMap(),
                                       tgt_cloud->points[pointIdxNKNSearch[0]].getVector3fMap(), voxel_map_tgt);
            total_residual += RobustKernel(residual, threshold, config.robust_kernel);
            intersect_num++;
        }
        total_residual = intersect_num > 0 ? total_residual / intersect_num : INFINITY;
//        LOG(INFO) << "total_residual: " << total_residual << ", transformed_src: " << transformed_src->size() << ", valid_num: " << valid_num;
        if (total_residual < best_pair.first) {
            best_pair = std::make_pair(total_residual, tf);
        }
        last_candidate = tf;
    }
    return std::make_pair(best_pair.first < INFINITY, best_pair.second);
}

void AdaptiveDownsample(const VoxelMap &voxel_map, int sample_num, std::vector<Eigen::Vector3f> &sample_pts) {
    sample_pts.reserve(voxel_map.size() * sample_num);
    for (auto voxel: voxel_map) {
        if (sample_num == 1) {
            sample_pts.push_back(voxel.second->center().cast<float>());
        } else {
            int pt_size = voxel.second->cloud()->size();
            if (pt_size > sample_num) {
                int step = pt_size / sample_num;
                for (int i = 0; i < sample_num; ++i) {
                    sample_pts.push_back(voxel.second->cloud()->points[i * step].getVector3fMap());
                }
            } else {
                for (int i = 0; i < pt_size; ++i) {
                    sample_pts.push_back(voxel.second->cloud()->points[i].getVector3fMap());
                }
            }
        }
    }
}

std::pair<bool, Eigen::Matrix4d> GeometryVerifyNanoFlann(const VoxelMap &voxel_map_src,
                                                         const VoxelMap &voxel_map_tgt,
                                                         const std::vector<Eigen::Matrix4d> &candidates) {

    if (candidates.size() == 1) {
        return std::make_pair(true, candidates[0]);
    }

    std::vector<Eigen::Vector3f> src_pts, tgt_pts, transformed_src_pts;
    AdaptiveDownsample(voxel_map_src, 5, src_pts);
    AdaptiveDownsample(voxel_map_tgt, 1, tgt_pts);

    KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3f>, float> kdtree(3, tgt_pts, 10, 1);

    float threshold = config.voxel_resolution[0];
    std::pair<float, Eigen::Matrix4d> best_pair = std::make_pair(INFINITY, candidates[0]);

    Eigen::Matrix4f last_candidate = Eigen::Matrix4f::Identity();
    for (int i = 0; i < candidates.size(); ++i) {
        Eigen::Matrix4f tf = candidates[i].cast<float>();
        if (i > 0 && (tf - last_candidate).norm() < 0.1) continue;
        int intersect_num = 0;
        float total_residual = 0.0, residual;
        robot_utils::transformPointCloud(src_pts, transformed_src_pts, tf);
        for (int i = 0; i < transformed_src_pts.size(); ++i) {
            std::vector<size_t> ret_indexes(1);
            std::vector<float> out_dists_sqr(1);
            nanoflann::KNNResultSet<float> resultSet(1);
            resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
            kdtree.index->findNeighbors(resultSet, &transformed_src_pts[i][0]);

            if (ret_indexes[0] < 0 || ret_indexes[0] >= tgt_pts.size()) continue;
            residual = ComputeResidual(transformed_src_pts[i], tgt_pts[ret_indexes[0]], voxel_map_tgt);
            total_residual += RobustKernel(residual, threshold, config.robust_kernel);
            intersect_num++;
        }
        total_residual = intersect_num > 0 ? total_residual / intersect_num : INFINITY;
//        LOG(INFO) << "total_residual: " << total_residual << ", transformed_src: " << transformed_src->size() << ", valid_num: " << valid_num;
        if (total_residual < best_pair.first) {
            best_pair = std::make_pair(total_residual, tf.cast<double>());
        }
        last_candidate = tf;
    }
    return std::make_pair(best_pair.first < INFINITY, best_pair.second);
}

Eigen::Matrix4d GeometryVerify(typename pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                               typename pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                               const std::vector<Eigen::Matrix4d> &candidates) {

    if (candidates.size() == 1) {
        return candidates[0];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
    voxelize(src_cloud, src_cloud_down, 1.0);
    voxelize(tgt_cloud, tgt_cloud_down, 1.0);

    // compute the chamfer distance
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(tgt_cloud_down);

    Eigen::Matrix4d best_pose = Eigen::Matrix4d::Identity();
    float threshold = config.voxel_resolution[0];
    double best_score = INFINITY;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto tf: candidates) {
        double total_residual = 0.0, dist;
        pcl::transformPointCloud(*src_cloud_down, *transformed_src, tf);
        for (int i = 0; i < transformed_src->size(); ++i) {
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
            tree->nearestKSearch(transformed_src->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            total_residual += RobustKernel(pointNKNSquaredDistance[0], threshold, "dcs");
        }
        total_residual = transformed_src->size() > 0 ? total_residual / transformed_src->size() : INFINITY;
        if (total_residual < best_score) {
            best_score = total_residual;
            best_pose = tf;
        }
    }
    return best_pose;
}