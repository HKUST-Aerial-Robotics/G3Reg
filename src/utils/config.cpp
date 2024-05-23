/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "utils/config.h"
#include <glog/logging.h>
#include "global_definition/global_definition.h"
#include <filesystem>

namespace g3reg {

    namespace config {

        // benchmark config
        bool verbose = false;
        std::string project_path = g3reg::WORK_SPACE_PATH;
        std::string config_file, dataset_root, label_dir, test_file, dataset_name, log_dir;
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
        double volume_chi2;
        clique_solver::VertexInfo vertex_info;
        //RANSAC
        double ransac_max_iterations, ransac_inlier_threshold, ransac_inliers_to_end;

        // Evaluation
        double tp_thresh, trans_thresh, rot_thresh;

        std::vector<double> get_vec(const YAML::Node &node, const std::string &father_key, const std::string &key) {
            std::vector<double> vec;
            if (!node[father_key] || !node[father_key][key]) {
                if (verbose) {
                    LOG(INFO) << "Key " << father_key << "/" << key << " not found, using default value: ";
                }
                return vec;
            }
            for (const auto &item: node[father_key][key]) {
                vec.push_back(item.as<double>());
            }
            if (verbose) {
                LOG(INFO) << "Key " << father_key << "/" << key << " found, using value: " << vec.size();
            }
            return vec;
        }

        void readParamsPlane(std::string sensor_dir);

        void readParamsFPFH(std::string sensor_dir);

        void readParameters(std::string config_file_, char **argv) {

            YAML::Node config_node;
            if (argv != nullptr) {
                config_file = config_file_;
                LOG(INFO) << "config_path: " << config_file;
                std::ifstream fin(config_file);
                if (!fin) {
                    std::cout << "config_file: " << config_file << " not found." << std::endl;
                    return;
                }
                config_node = YAML::LoadFile(config_file);
            }

            verbose = get(config_node, "verbose", true);
            dataset_root = get(config_node, "dataset", "dataset_root", std::string(""));
            dataset_name = get(config_node, "dataset", "name", std::string("kitti-10m"));
            label_dir = get(config_node, "dataset", "label_dir", std::string("labels"));
            std::string sensor_dir = get(config_node, "dataset", "sensor_dir", std::string(""));
            test_file = argv != nullptr ? argv[2] : "";

            sensor_dir = project_path + "/" + sensor_dir;
            dcvc_file = sensor_dir + "/dcvc.yaml";
            travel_file = sensor_dir + "/travel.yaml";
            if (verbose) {
                LOG(INFO) << "dcvc_file: " << dcvc_file;
                LOG(INFO) << "travel_file: " << travel_file;
            }

            min_range = get(config_node, "dataset", "min_range", 0.5);
            max_range = get(config_node, "dataset", "max_range", 120.0);
            min_cluster_size = get(config_node, "dataset", "min_cluster_size", 20);
            ds_resolution = get(config_node, "dataset", "ds_resolution", 0.5);

            front_end = get(config_node, "front_end", std::string("gem"));
            cluster_mtd = get(config_node, "cluster_mtd", std::string("dcvc"));
            back_end = get(config_node, "back_end", std::string("pagor"));
            tf_solver = get(config_node, "tf_solver", std::string("gnc"));
            verify_mtd = get(config_node, "verify_mtd", std::string("voxel"));
            robust_kernel = get(config_node, "robust_kernel", std::string("dcs"));

            // evaluation
            tp_thresh = get(config_node, "evaluation", "tp_thresh", 0.5);
            trans_thresh = get(config_node, "evaluation", "trans_thresh", 2.0);
            rot_thresh = get(config_node, "evaluation", "rot_thresh", 5.0);

            // association parameter
            assoc_method = get(config_node, "association", "method", std::string("wasserstein"));
            assoc_topk = get(config_node, "association", "topK", 20);
            num_clusters = get(config_node, "association", "num_clusters", 50);
            num_planes = get(config_node, "association", "num_planes", 50);
            num_lines = get(config_node, "association", "num_lines", 50);
            max_corrs = get(config_node, "association", "max_corrs", 3000);

            use_pseudo_cov = get(config_node, "use_pseudo_cov", true);
            use_bbox_center = get(config_node, "use_bbox_center", false);
            plane_aided = get(config_node, "plane_aided", true);
            grad_pmc = get(config_node, "grad_pmc", true);
            volume_chi2 = get(config_node, "volume_chi2", 7.815);

            // vertex parameter
            std::string vertex_type = get(config_node, "vertex", "type", std::string("point"));
            if (vertex_type == "gaussian") {
                vertex_info.type = clique_solver::VertexType::GAUSSIAN;
                use_pseudo_cov = false;
            } else if (vertex_type == "point") {
                vertex_info.type = clique_solver::VertexType::POINT;
                use_pseudo_cov = false;
            } else if (vertex_type == "point_ratio") {
                vertex_info.type = clique_solver::VertexType::POINT_RATIO;
                use_pseudo_cov = false;
            } else if (vertex_type == "ellipse") {
                vertex_info.type = clique_solver::VertexType::ELLIPSE;
            } else {
                LOG(ERROR) << "vertex type error";
            }
            vertex_info.noise_bound_vec = get_vec(config_node, "vertex", "noise_bound_vec");
            num_graphs = vertex_info.noise_bound_vec.size() == 0 ? 1 : vertex_info.noise_bound_vec.size();

            ransac_max_iterations = get(config_node, "ransac", "max_iterations", 100000);
            ransac_inlier_threshold = get(config_node, "ransac", "inlier_threshold", 0.5);
            ransac_inliers_to_end = get(config_node, "ransac", "inliers_to_end", 0.5);

            readParamsFPFH(sensor_dir);
            readParamsPlane(sensor_dir);
        }

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
    }

    void InitGLOG(std::string config_path, char **argv) {
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "config_path: " << config_path << std::endl;
            std::cerr << "config file not found." << std::endl;
            return;
        }
        YAML::Node config_node = YAML::LoadFile(config_path);
        std::string config_name = config_path.substr(config_path.find_last_of('/') + 1,
                                                     config_path.find_last_of('.') - config_path.find_last_of('/') - 1);
        std::string dataset_name = config::get(config_node, "dataset", "name", std::string("kitti-10m"));
        std::string test_split = "";
        if (argv != nullptr) {
            test_split = std::filesystem::path(argv[2]).stem().string();
        } else {
            test_split = "demo";
        }

        config::log_dir = g3reg::WORK_SPACE_PATH + "/Log/";
        FileManager::CreateDirectory(config::log_dir);
        config::log_dir = config::log_dir + dataset_name + "/";
        FileManager::CreateDirectory(config::log_dir);
        config::log_dir = config::log_dir + config_name;
        FileManager::CreateDirectory(config::log_dir);
        config::log_dir = config::log_dir + "/" + test_split;
        FileManager::CreateDirectory(config::log_dir);

        google::InitGoogleLogging("");
        std::string log_file = config::log_dir + "/log-";
        google::SetLogDestination(google::INFO, log_file.c_str());

        FLAGS_log_dir = config::log_dir;
        FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr = true;
        FLAGS_log_prefix = false;
        FLAGS_logbufsecs = 0;
//    FLAGS_timestamp_in_logfile_name = true;
    }
}