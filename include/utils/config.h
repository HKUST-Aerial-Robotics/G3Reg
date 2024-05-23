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

namespace g3reg {

    class Config {
    public:
        // benchmark config
        bool verbose;
        std::string project_path;
        std::string dataset_root, label_dir, test_file, dataset_name, log_dir, sensor_dir;
        std::string dcvc_file, travel_file;

        std::string front_end, back_end, tf_solver, cluster_mtd, verify_mtd, robust_kernel;

        // Front Engd
        double max_range, min_range, ds_resolution;
        int min_cluster_size;
        // Association
        std::string assoc_method;
        int assoc_topk, num_clusters, num_planes, num_lines, max_corrs;
        // GEM
        bool use_pseudo_cov, plane_aided, use_bbox_center, grad_pmc;

        // Back End
        // PAGOR
        int num_graphs = 0;
        std::string vertex_type;
        double volume_chi2;
        clique_solver::VertexInfo vertex_info;
        //RANSAC
        double ransac_max_iterations, ransac_inlier_threshold, ransac_inliers_to_end;

        // Evaluation
        double tp_thresh, trans_thresh, rot_thresh;

        double plane_resolution, plane_distance_thresh, plane_normal_thresh, eigenvalue_thresh;
        Eigen::Vector3f voxel_resolution;

        void readParamsPlane(std::string sensor_dir) {
            std::string plane_file = sensor_dir + "/plane.yaml";
            if (verbose) {
                LOG(INFO) << "plane_file: " << plane_file;
            }
            YAML::Node config_node;
            if (std::ifstream(plane_file)) {
                config_node = YAML::LoadFile(plane_file);
            }
            plane_resolution = get(config_node, "plane_extraction", "resolution", 1.0);
            plane_distance_thresh = get(config_node, "plane_extraction", "distance_thresh", 0.2);
            plane_normal_thresh = get(config_node, "plane_extraction", "normal_thresh", 0.95);
            eigenvalue_thresh = get(config_node, "plane_extraction", "eigenvalue_thresh", 30);
            voxel_resolution = Eigen::Vector3f(plane_resolution, plane_resolution, plane_resolution);
        }

        double normal_radius, fpfh_radius;

        void readParamsFPFH(std::string sensor_dir) {
            std::string fpfh_file = sensor_dir + "/fpfh.yaml";
            if (verbose) {
                LOG(INFO) << "fpfh_file: " << fpfh_file;
            }
            YAML::Node config_node;
            if (std::ifstream(fpfh_file)) {
                config_node = YAML::LoadFile(fpfh_file);
            }
            normal_radius = get(config_node, "fpfh", "normal_radius", 1.0);
            fpfh_radius = get(config_node, "fpfh", "fpfh_radius", 2.5);
        }

        template<typename T>
        T get(const YAML::Node &node, const std::string &key, const T &default_value) {
            if (!node[key]) {
                if (verbose) {
                    LOG(INFO) << "Key " << key << " not found, using default value: " << default_value;
                }
                return default_value;
            }
            T value = node[key].as<T>();
            if (verbose) {
                LOG(INFO) << "Key " << key << " found, using value: " << value;
            }
            return value;
        }

        template<typename T>
        T get(const YAML::Node &node, const std::string &father_key, const std::string &key, const T &default_value) {
            if (!node[father_key] || !node[father_key][key]) {
                if (verbose) {
                    LOG(INFO) << "Key " << father_key << "/" << key << " not found, using default value: "
                              << default_value;
                }
                return default_value;
            }
            T value = node[father_key][key].as<T>();
            if (verbose) {
                LOG(INFO) << "Key " << father_key << "/" << key << " found, using value: " << value;
            }
            return value;
        }

        std::vector<double> get_vec(const YAML::Node &node, const std::string &father_key, const std::string &key,
                                    const std::vector<double> &default_value) {
            if (!node[father_key] || !node[father_key][key]) {
                if (verbose) {
                    LOG(INFO) << "Key " << father_key << "/" << key << " not found, using default value: ";
                }
                return default_value;
            }
            std::vector<double> vec;
            for (const auto &item: node[father_key][key]) {
                vec.push_back(item.as<double>());
            }
            if (verbose) {
                LOG(INFO) << "Key " << father_key << "/" << key << " found, using value: " << vec.size();
            }
            return vec;
        }

    public:
        Config();

        void reset_config();

        void load_config(const std::string &config_file = "", char **argv = nullptr);
    };

    extern Config config;

    void InitGLOG(const std::string &config_path, char **argv);
}

#endif //SRC_CONFIG_H
