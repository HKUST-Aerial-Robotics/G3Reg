/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_EVALUATION_H
#define SRC_EVALUATION_H
#include "utils/config.h"
#include "robot_utils/lie_utils.h"

typedef struct {
    double recall = 0;
    double precision = 0;
    double f1 = 0;
    double tp = 0;
    double time = 0;
    int assoc_num = 0;
}FeatureMetric;

class FRGresult{
public:
    FRGresult(){
        tf = Eigen::Matrix4d::Identity();
        plane_inliers = 0;
        line_inliers = 0;
        cluster_inliers = 0;
        candidates.resize(config::num_graphs, Eigen::Matrix4d::Identity());
        valid = true;
    }
public:
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Matrix4d> candidates;
    int plane_inliers, line_inliers, cluster_inliers;
    clique_solver::Association inliers;
    double feature_time = 0;
    double tf_solver_time = 0;
    double clique_time = 0, graph_time = 0;
    double verify_time = 0;
    double total_time = 0;
    bool valid = true;
};

class Evaluation {
public:

    double num, success_num, success_num_upper, success_rate, success_rate_upper;
    double feature_time, feature_time_avg;
    double tf_solver_time, solver_time_avg;
    double clique_time, clique_time_avg;
    double graph_time, graph_time_avg;
    double verify_time, verify_time_avg;
    double total_time, total_time_avg;
    double plane_inliers, plane_inliers_avg;
    double line_inliers, line_inliers_avg;
    double cluster_inliers, cluster_inliers_avg;
    double rot_err_mae, rot_err_rmse;
    double trans_err_mae, trans_err_rmse;
    std::vector<double> rot_errs, trans_errs, feature_times, clique_times, graph_times, verify_times, solver_times, runtimes;
    
    Evaluation(){
        num = success_num = success_num_upper = success_rate = success_rate_upper = 0;
        feature_time = feature_time_avg = 0;
        tf_solver_time = solver_time_avg = 0;
        clique_time = clique_time_avg = 0;
        graph_time = graph_time_avg = 0;
        verify_time = verify_time_avg = 0;
        total_time = total_time_avg = 0;
        plane_inliers = plane_inliers_avg = 0;
        line_inliers = line_inliers_avg = 0;
        cluster_inliers = cluster_inliers_avg = 0;
        rot_err_mae = rot_err_rmse = 0;
        trans_err_mae = trans_err_rmse = 0;
        rot_errs.clear();
        trans_errs.clear();
		runtimes.clear();
		feature_times.clear(); clique_times.clear(); graph_times.clear(); verify_times.clear(); solver_times.clear();
    }
    
    std::pair<bool, bool> update(FRGresult solution, const Eigen::Matrix4d& tf_gt){
        num++;
        feature_time = solution.feature_time;
        tf_solver_time = solution.tf_solver_time;
        clique_time = solution.clique_time;
        graph_time = solution.graph_time;
        verify_time = solution.verify_time;
        total_time = solution.total_time;
        plane_inliers = solution.plane_inliers;
        line_inliers = solution.line_inliers;
        cluster_inliers = solution.cluster_inliers;
        
        feature_time_avg = (feature_time_avg * (num - 1) + feature_time) / num;
        solver_time_avg = (solver_time_avg * (num - 1) + tf_solver_time) / num;
        clique_time_avg = (clique_time_avg * (num - 1) + clique_time) / num;
        graph_time_avg = (graph_time_avg * (num - 1) + graph_time) / num;
        verify_time_avg = (verify_time_avg * (num - 1) + verify_time) / num;
        total_time_avg = (total_time_avg * (num - 1) + total_time) / num;
        plane_inliers_avg = (plane_inliers_avg * (num - 1) + plane_inliers) / num;
        line_inliers_avg = (line_inliers_avg * (num - 1) + line_inliers) / num;
        cluster_inliers_avg = (cluster_inliers_avg * (num - 1) + cluster_inliers) / num;
		runtimes.push_back(total_time); feature_times.push_back(feature_time); clique_times.push_back(clique_time);
		graph_times.push_back(graph_time); verify_times.push_back(verify_time); solver_times.push_back(tf_solver_time);

        bool success_flag_upper = false, success_flag = false;
        if (is_succ(solution.tf, tf_gt)){
            success_num+=100;
            success_flag = true;
        }
        success_rate = success_num / num;
        for (int level = 0; level < solution.candidates.size(); level++){
            auto tf_est = solution.candidates[level];
            if (is_succ(tf_est, tf_gt)){
                success_num_upper+=100;
                success_flag_upper = true;
//                LOG(INFO) << "level: " << level;
                break;
            }
        }
        success_rate_upper = success_num_upper / num;
        return std::make_pair(success_flag, success_flag_upper);
    }

    bool is_succ(const Eigen::Matrix4d& tf_est, const Eigen::Matrix4d& tf_gt){
        Eigen::Matrix3d rot_err = tf_gt.block<3, 3>(0, 0).transpose() * tf_est.block<3, 3>(0, 0);
        Eigen::Vector3d trans_err = tf_gt.block<3, 3>(0, 0).transpose() * (tf_est.block<3, 1>(0, 3) - tf_gt.block<3, 1>(0, 3));
        if (robot_utils::R2Angle(rot_err, true) < config::rot_thresh && trans_err.norm() < config::trans_thresh){
            rot_errs.push_back(abs(robot_utils::R2Angle(rot_err, true)));
            trans_errs.push_back(trans_err.norm());
            return true;
        } else {
            return false;
        }
    }
	
	void writeStatistics(std::string& filename, std::vector<double>& data){
		std::ofstream file(filename);
		for (auto d:data){
			file << d << std::endl;
		}
		file.close();
	}
	
	void saveStatistics(const std::string& directory, int seq=-1){
		std::string err_dir = directory + "/errors";
		FileManager::CreateDirectory(err_dir);
		std::string rot_filename = err_dir + "/rot_errs" + (seq==-1?"":std::to_string(seq)) + ".txt";
		std::string trans_filename = err_dir + "/trans_errs" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(rot_filename, rot_errs);
		writeStatistics(trans_filename, trans_errs);
		
		std::string runtime_dir = directory + "/runtimes";
		FileManager::CreateDirectory(runtime_dir);
		std::string runtime_filename = runtime_dir + "/runtimes" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(runtime_filename, runtimes);
		std::string feature_time_filename = runtime_dir + "/feature_times" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(feature_time_filename, feature_times);
		std::string clique_time_filename = runtime_dir + "/clique_times" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(clique_time_filename, clique_times);
		std::string graph_time_filename = runtime_dir + "/graph_times" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(graph_time_filename, graph_times);
		std::string verify_time_filename = runtime_dir + "/verify_times" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(verify_time_filename, verify_times);
		std::string solver_time_filename = runtime_dir + "/solver_times" + (seq==-1?"":std::to_string(seq)) + ".txt";
		writeStatistics(solver_time_filename, solver_times);
	}
	
    void computePoseErr(){
        // 计算旋转误差的 MAE 和 RMSE
        if (!rot_errs.empty()) {
            double rot_err_sum = std::accumulate(rot_errs.begin(), rot_errs.end(), 0.0);
            rot_err_mae = rot_err_sum / rot_errs.size();

            double rot_err_sq_sum = 0.0;
            for (double err : rot_errs) {
                rot_err_sq_sum += std::pow(err - rot_err_mae, 2);
            }
            rot_err_rmse = std::sqrt(rot_err_sq_sum / rot_errs.size());
        }

        // 计算平移误差的 MAE 和 RMSE
        if (!trans_errs.empty()) {
            double trans_err_sum = std::accumulate(trans_errs.begin(), trans_errs.end(), 0.0);
            trans_err_mae = trans_err_sum / trans_errs.size();

            double trans_err_sq_sum = 0.0;
            for (double err: trans_errs) {
                trans_err_sq_sum += std::pow(err - trans_err_mae, 2);
            }
            trans_err_rmse = std::sqrt(trans_err_sq_sum / trans_errs.size());
        }
    }
};

#include <pcl/point_cloud.h>
FeatureMetric FrontEndEval(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                           std::tuple<int, int, int> pair_info, Eigen::Matrix4d T_gt);

#endif //SRC_EVALUATION_H
