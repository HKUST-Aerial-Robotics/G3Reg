/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <glog/logging.h>
#include "robot_utils/tic_toc.h"
#include <pcl/point_types.h>
#include "robot_utils/file_manager.h"
#include "front_end/graph_vertex.h"
#include <clique_solver/graph.h>
#include "back_end/teaser/registration.h"

namespace config {

    extern std::string dcvc_file, travel_file;
    extern std::string front_end, back_end, tf_solver, cluster_mtd, verify_mtd, robust_kernel;
    extern bool use_pseudo_cov, plane_aided, use_bbox_center, grad_pmc;
    extern double volume_chi2;
    extern clique_solver::VertexInfo vertex_info;
    extern int num_graphs;
    extern double ransac_max_iterations, ransac_inlier_threshold, ransac_inliers_to_end;

    // benchmark config
    extern std::string project_path, log_dir;
    extern std::string config_file;

    extern std::string assoc_method;
    extern int assoc_topk, num_clusters, num_planes, num_lines, max_corrs;

    extern double tp_thresh;
    extern double trans_thresh;
    extern double rot_thresh;

    extern double normal_radius;
    extern double fpfh_radius;
    extern double ds_resolution;

    extern double max_range;
    extern double min_range;
    extern int min_cluster_size;
    extern std::string dataset_root, label_dir, test_file, dataset_name;

    extern double plane_resolution;
    extern double plane_distance_thresh;
    extern double plane_normal_thresh;
    extern double eigenvalue_thresh;
    extern Eigen::Vector3f voxel_resolution;

    void readParameters(std::string config_file = "", char **argv = nullptr);

    template<typename T>
    T get(const YAML::Node &node, const std::string &key, const T &default_value) {
        if (!node[key]) {
            LOG(INFO) << "Key " << key << " not found, using default value: " << default_value;
            return default_value;
        }
        T value = node[key].as<T>();
        LOG(INFO) << "Key " << key << " found, using value: " << value;
        return value;
    }

    template<typename T>
    T get(const YAML::Node &node, const std::string &father_key, const std::string &key, const T &default_value) {
        if (!node[father_key] || !node[father_key][key]) {
            LOG(INFO) << "Key " << father_key << "/" << key << " not found, using default value: " << default_value;
            return default_value;
        }
        T value = node[father_key][key].as<T>();
        LOG(INFO) << "Key " << father_key << "/" << key << " found, using value: " << value;
        return value;
    }
}

void InitGLOG(std::string config_path, char **argv);

#endif //SRC_CONFIG_H
