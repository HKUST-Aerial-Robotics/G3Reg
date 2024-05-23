#include "utils/config.h"
#include <iostream>
#include <string>
#include <Eigen/Core>
#include "datasets/datasets_init.h"
#include "back_end/reglib.h"

using namespace std;
using namespace clique_solver;
using namespace g3reg;

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "Usage: reg_bm config_file test_file" << std::endl;
        return -1;
    }
    std::string config_path = config.project_path + "/" + argv[1];
    InitGLOG(config_path, argv);
    LOG(INFO) << "Test file: " << config.test_file;
    config.load_config(config_path, argv);

    DataLoader::Ptr dataloader_ptr = CreateDataLoader();
    auto &items = dataloader_ptr->items;

    int pair_idx = 0;
    Evaluation eval;
    std::map<int, Evaluation> eval_seq_map;
    for (auto &item: items) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud = dataloader_ptr->GetCloud(config.dataset_root, item.seq,
                                                                                 item.src_idx);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud = dataloader_ptr->GetCloud(config.dataset_root, item.seq_db,
                                                                                 item.tgt_idx);
        FRGresult solution = g3reg::GlobalRegistration(src_cloud, tgt_cloud,
                                                       std::make_tuple(item.seq, item.src_idx, item.tgt_idx));

        bool success_flag_upper = false, success_flag = false;
        std::tie(success_flag, success_flag_upper) = eval.update(solution, item.pose);
        if (eval_seq_map.find(item.seq) == eval_seq_map.end()) {
            eval_seq_map[item.seq] = Evaluation();
        }
        eval_seq_map[item.seq].update(solution, item.pose);
        pair_idx++;

        LOG(INFO) << std::fixed << std::setprecision(2) << "(" << item.seq << ", " << item.src_idx << ")/("
                  << item.seq_db << ", " << item.tgt_idx << ")" << ", " << pair_idx << "/" << items.size()
                  << ", succ: " << eval.success_rate << "/" << eval.success_rate_upper << "/" << success_flag << "/"
                  << success_flag_upper
                  << ", time front/graph/clique/solve_tf/verify/total: " << eval.feature_time << "/" << eval.graph_time
                  << "/" << eval.clique_time
                  << "/" << eval.tf_solver_time << "/" << eval.verify_time << "/" << eval.total_time << " ms"
                  << ", inliers: " << eval.plane_inliers << "/" << eval.line_inliers << "/" << eval.cluster_inliers
                  << ", ol: " << item.overlap << ", trans:" << item.pose.block<3, 1>(0, 3).norm();
    }

    // compute average
    LOG(INFO) << "Evaluation for all sequences: ";
    eval.computePoseErr();
    eval.saveStatistics(config.log_dir);
    LOG(INFO) << "Succ ratio: " << eval.success_rate << "/" << eval.success_rate_upper
              << " Time front/graph/clique/solve_tf/verify/total: " << eval.feature_time_avg << "/"
              << eval.graph_time_avg << "/" << eval.clique_time_avg
              << "/" << eval.solver_time_avg << "/" << eval.verify_time_avg << "/" << eval.total_time_avg << " ms"
              << ", inliers: " << eval.plane_inliers_avg << "/" << eval.line_inliers_avg << "/"
              << eval.cluster_inliers_avg
              << "Rot MAE: " << eval.rot_err_mae << ", RMSE: " << eval.rot_err_rmse << ", translation MAE: "
              << eval.trans_err_mae << ", RMSE: " << eval.trans_err_rmse;
    LOG(INFO) << "Evaluation for each sequence: ";
    for (auto eval_dict: eval_seq_map) {
        int seq = eval_dict.first;
        Evaluation eval_seq = eval_dict.second;
        eval_seq.computePoseErr();
        eval_seq.saveStatistics(config.log_dir, seq);
        LOG(INFO) << "Seq: " << seq << "(#items: " << eval_seq.num << ")" << ", Succ ratio: " << eval_seq.success_rate
                  << "/" << eval_seq.success_rate_upper
                  << " Time front/graph/clique/solve_tf/verify/total: " << eval_seq.feature_time_avg << "/"
                  << eval.graph_time_avg << "/" << eval.clique_time_avg
                  << "/" << eval_seq.solver_time_avg << "/" << eval_seq.verify_time_avg << "/"
                  << eval_seq.total_time_avg << " ms"
                  << ", inliers: " << eval_seq.plane_inliers_avg << "/" << eval_seq.line_inliers_avg << "/"
                  << eval_seq.cluster_inliers_avg
                  << "Rot MAE: " << eval_seq.rot_err_mae << ", RMSE: " << eval_seq.rot_err_rmse << ", translation MAE: "
                  << eval_seq.trans_err_mae << ", RMSE: " << eval_seq.trans_err_rmse;
    }
    return 0;
}