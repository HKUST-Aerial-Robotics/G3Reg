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

        std::string front_end, back_end, tf_solver, cluster_mtd;

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

        // Transformation Verification
        std::string verify_mtd, robust_kernel;

        // Evaluation
        double tp_thresh, trans_thresh, rot_thresh;

        double plane_resolution, plane_distance_thresh, plane_normal_thresh, eigenvalue_thresh;

        double normal_radius, fpfh_radius;

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

        Config();

        void reset_config();

        void load_config(const std::string &config_file = "", char **argv = nullptr);

        void set_noise_bounds(const std::vector<double> &val);
    };

    extern Config config;

    void InitGLOG(const std::string &config_path, char **argv);
}

#endif //SRC_CONFIG_H
