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

    Config config;

    Config::Config() {
        reset_config();
    }

    void Config::reset_config() {
        verbose = false;

        dataset_root = "";
        dataset_name = "kitti-10m";
        label_dir = "labels";
        sensor_dir = "";
        test_file = "";
        project_path = g3reg::WORK_SPACE_PATH;

        std::string abs_sensor_dir = project_path + "/" + sensor_dir;
        dcvc_file = abs_sensor_dir + "/dcvc.yaml";
        travel_file = abs_sensor_dir + "/travel.yaml";

        min_range = 0.5;
        max_range = 120.0;
        min_cluster_size = 20;
        ds_resolution = 0.5;

        front_end = "gem";
        cluster_mtd = "dcvc";
        back_end = "pagor";
        tf_solver = "gnc";
        verify_mtd = "voxel";
        robust_kernel = "dcs";

        // evaluation
        tp_thresh = 0.5;
        trans_thresh = 2.0;
        rot_thresh = 5.0;

        // association parameter
        assoc_method = "wasserstein";
        assoc_topk = 20;
        num_clusters = 50;
        num_planes = 50;
        num_lines = 50;
        max_corrs = 3000;

        use_pseudo_cov = true;
        use_bbox_center = false;
        plane_aided = true;
        grad_pmc = true;
        volume_chi2 = 7.815;

        // vertex parameter
        vertex_type = "point";
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
        vertex_info.noise_bound_vec = {0.2};
        num_graphs = vertex_info.noise_bound_vec.size() == 0 ? 1 : vertex_info.noise_bound_vec.size();

        ransac_max_iterations = 100000;
        ransac_inlier_threshold = 0.5;
        ransac_inliers_to_end = 0.5;

        normal_radius = 1.0;
        fpfh_radius = 2.5;

        plane_resolution = 1.0;
        plane_distance_thresh = 0.2;
        plane_normal_thresh = 0.95;
        eigenvalue_thresh = 30;
        voxel_resolution = Eigen::Vector3f(plane_resolution, plane_resolution, plane_resolution);
    }


    void Config::load_config(const std::string &config_file, char **argv) {

        YAML::Node config_node;
        if (!config_file.empty()) {
            LOG(INFO) << "config_path: " << config_file;
            std::ifstream fin(config_file);
            if (!fin) {
                std::cout << "config_file: " << config_file << " not found." << std::endl;
                return;
            }
            config_node = YAML::LoadFile(config_file);
        }

        verbose = get(config_node, "verbose", true);
        dataset_root = get(config_node, "dataset", "dataset_root", dataset_root);
        dataset_name = get(config_node, "dataset", "name", dataset_name);
        label_dir = get(config_node, "dataset", "label_dir", label_dir);
        sensor_dir = get(config_node, "dataset", "sensor_dir", sensor_dir);
        test_file = argv != nullptr ? argv[2] : "";

        std::string abs_sensor_dir = project_path + "/" + sensor_dir;
        dcvc_file = abs_sensor_dir + "/dcvc.yaml";
        travel_file = abs_sensor_dir + "/travel.yaml";
        std::string fpfh_file = abs_sensor_dir + "/fpfh.yaml";
        std::string plane_file = abs_sensor_dir + "/plane.yaml";
        if (verbose) {
            LOG(INFO) << "dcvc_file: " << dcvc_file;
            LOG(INFO) << "travel_file: " << travel_file;
            LOG(INFO) << "plane_file: " << plane_file;
            LOG(INFO) << "fpfh_file: " << fpfh_file;
        }

        min_range = get(config_node, "dataset", "min_range", min_range);
        max_range = get(config_node, "dataset", "max_range", max_range);
        min_cluster_size = get(config_node, "dataset", "min_cluster_size", min_cluster_size);
        ds_resolution = get(config_node, "dataset", "ds_resolution", ds_resolution);

        front_end = get(config_node, "front_end", front_end);
        cluster_mtd = get(config_node, "cluster_mtd", cluster_mtd);
        back_end = get(config_node, "back_end", back_end);
        tf_solver = get(config_node, "tf_solver", tf_solver);
        verify_mtd = get(config_node, "verify_mtd", verify_mtd);
        robust_kernel = get(config_node, "robust_kernel", robust_kernel);

        // evaluation
        tp_thresh = get(config_node, "evaluation", "tp_thresh", tp_thresh);
        trans_thresh = get(config_node, "evaluation", "trans_thresh", trans_thresh);
        rot_thresh = get(config_node, "evaluation", "rot_thresh", rot_thresh);

        // association parameter
        assoc_method = get(config_node, "association", "method", assoc_method);
        assoc_topk = get(config_node, "association", "topK", assoc_topk);
        num_clusters = get(config_node, "association", "num_clusters", num_clusters);
        num_planes = get(config_node, "association", "num_planes", num_planes);
        num_lines = get(config_node, "association", "num_lines", num_lines);
        max_corrs = get(config_node, "association", "max_corrs", max_corrs);

        use_pseudo_cov = get(config_node, "use_pseudo_cov", use_pseudo_cov);
        use_bbox_center = get(config_node, "use_bbox_center", use_bbox_center);
        plane_aided = get(config_node, "plane_aided", plane_aided);
        grad_pmc = get(config_node, "grad_pmc", grad_pmc);
        volume_chi2 = get(config_node, "volume_chi2", volume_chi2);

        // vertex parameter
        vertex_type = get(config_node, "vertex", "type", vertex_type);
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
        vertex_info.noise_bound_vec = get_vec(config_node, "vertex", "noise_bound_vec", vertex_info.noise_bound_vec);
        num_graphs = vertex_info.noise_bound_vec.size() == 0 ? 1 : vertex_info.noise_bound_vec.size();

        ransac_max_iterations = get(config_node, "ransac", "max_iterations", ransac_max_iterations);
        ransac_inlier_threshold = get(config_node, "ransac", "inlier_threshold", ransac_inlier_threshold);
        ransac_inliers_to_end = get(config_node, "ransac", "inliers_to_end", ransac_inliers_to_end);

        if (std::ifstream(fpfh_file)) {
            config_node = YAML::LoadFile(fpfh_file);
        }
        normal_radius = get(config_node, "fpfh", "normal_radius", normal_radius);
        fpfh_radius = get(config_node, "fpfh", "fpfh_radius", fpfh_radius);

        if (std::ifstream(plane_file)) {
            config_node = YAML::LoadFile(plane_file);
        }
        plane_resolution = get(config_node, "plane_extraction", "resolution", plane_resolution);
        plane_distance_thresh = get(config_node, "plane_extraction", "distance_thresh", plane_distance_thresh);
        plane_normal_thresh = get(config_node, "plane_extraction", "normal_thresh", plane_normal_thresh);
        eigenvalue_thresh = get(config_node, "plane_extraction", "eigenvalue_thresh", eigenvalue_thresh);
        voxel_resolution = Eigen::Vector3f(plane_resolution, plane_resolution, plane_resolution);
    }

    void InitGLOG(const std::string &config_path, char **argv) {
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "config_path: " << config_path << std::endl;
            std::cerr << "config file not found." << std::endl;
            return;
        }
        YAML::Node config_node = YAML::LoadFile(config_path);
        std::string config_name = config_path.substr(config_path.find_last_of('/') + 1,
                                                     config_path.find_last_of('.') - config_path.find_last_of('/') - 1);
        std::string dataset_name = config.get(config_node, "dataset", "name", std::string("kitti-10m"));
        std::string test_split;
        if (argv != nullptr) {
            test_split = std::filesystem::path(argv[2]).stem().string();
        } else {
            test_split = "demo";
        }

        config.log_dir = g3reg::WORK_SPACE_PATH + "/Log/";
        FileManager::CreateDirectory(config.log_dir);
        config.log_dir = config.log_dir + dataset_name + "/";
        FileManager::CreateDirectory(config.log_dir);
        config.log_dir = config.log_dir + config_name;
        FileManager::CreateDirectory(config.log_dir);
        config.log_dir = config.log_dir + "/" + test_split;
        FileManager::CreateDirectory(config.log_dir);

        google::InitGoogleLogging("");
        std::string log_file = config.log_dir + "/log-";
        google::SetLogDestination(google::INFO, log_file.c_str());

        FLAGS_log_dir = config.log_dir;
        FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr = true;
        FLAGS_log_prefix = false;
        FLAGS_logbufsecs = 0;
//    FLAGS_timestamp_in_logfile_name = true;
    }
}