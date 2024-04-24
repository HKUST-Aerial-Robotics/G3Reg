/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "front_end/fcgf.h"
#include "datasets/datasets_init.h"
#include <robot_utils/file_manager.h>

namespace fcgf{
    static ApolloData apollo_data;

    std::string GetFCGFDir(int seq){
        std::string dataset_name = config::dataset_name;
        std::transform(dataset_name.begin(), dataset_name.end(), dataset_name.begin(), ::tolower);
        if (dataset_name == "apollo"){
            std::string apollo_dir = apollo_data.apollo_sessions[seq];
            return FileManager::JoinPath(FileManager::JoinPath(config::dataset_root, apollo_dir), "correspondences");
        } else if (dataset_name == "kitti"){
            std::string fcgf_dir = (boost::format("%s/%02d/correspondences") % config::dataset_root % seq).str();
            return fcgf_dir;
        } else if (dataset_name == "kitti360"){
            std::string fcgf_dir = boost::str(boost::format("%s/data_3d_raw/2013_05_28_drive_%04d_sync/velodyne_points/correspondences") % config::dataset_root % seq);
            return fcgf_dir;
        } else {
            throw std::runtime_error("GetFCGFDir Unknown dataset type");
        }
    }

    clique_solver::Association matching(std::tuple<int, int, int> pair_info,
                                        std::vector<clique_solver::GraphVertex::Ptr> &src_nodes,
                                        std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes) {

        int seq = std::get<0>(pair_info);
        int src_id = std::get<1>(pair_info);
        int tgt_id = std::get<2>(pair_info);
        std::string fcgf_dir = GetFCGFDir(seq);
        std::string fcgf_corr_path = FileManager::JoinPath(fcgf_dir, std::to_string(src_id) + "_" + std::to_string(tgt_id) + ".txt");
        std::ifstream fcgf_corr_file(fcgf_corr_path);
        if (!fcgf_corr_file.is_open()) {
            LOG(ERROR) << "Cannot open file " << fcgf_corr_path;
            throw std::runtime_error("Cannot open file " + fcgf_corr_path);
        }
        // read the correspondences N x 6
        std::vector<Eigen::Vector3d> src_ds, tgt_ds;
        std::string line;
        while (std::getline(fcgf_corr_file, line)) {
            std::istringstream iss(line);
            Eigen::Vector3d src, tgt;
            iss >> src(0) >> src(1) >> src(2) >> tgt(0) >> tgt(1) >> tgt(2);
            src_ds.push_back(src);
            tgt_ds.push_back(tgt);
        }

        std::vector<int> indices;
        indices.reserve(src_ds.size());
        for(int i = 0; i < src_ds.size(); ++i) {
            indices.push_back(i);
        }
        if (indices.size() > config::max_corrs && config::back_end!="ransac"){
            // downsample the correspondences to 2000 without replacement
            indices.resize(config::max_corrs); // keep only the first 1000 elements
        }

        clique_solver::Association assoc = clique_solver::Association::Zero(indices.size(), 2);
        for (int i = 0; i < indices.size(); i++) {
            assoc(i, 0) = i;
            assoc(i, 1) = i;
        }

        // construct the matching node pairs
        src_nodes.clear();
        tgt_nodes.clear();
        src_nodes.reserve(indices.size());
        tgt_nodes.reserve(indices.size());

        for (int i = 0; i < indices.size(); i++) {
            const Eigen::Vector3d& center = src_ds[indices[i]];
            src_nodes.push_back(clique_solver::create_vertex(center, config::vertex_info));
            const Eigen::Vector3d& center_tgt = tgt_ds[indices[i]];
            tgt_nodes.push_back(clique_solver::create_vertex(center_tgt, config::vertex_info));
        }
        return assoc;
    }
}
