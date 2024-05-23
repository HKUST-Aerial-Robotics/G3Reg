/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "back_end/reglib.h"
#include "utils/evaluation.h"
#include "front_end/gem/ellipsoid.h"
#include "front_end/fpfh_utils.h"
#include "front_end/fcgf.h"
#include "back_end/pagor/pagor.h"
#include "back_end/ransac/ransac.h"
#include "back_end/mac3d/mac_reg.h"

using namespace std;
using namespace clique_solver;

namespace g3reg { //fast and robust global registration

    FRGresult GlobalRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt_cloud,
                                 std::tuple<int, int, int> pair_info) {

        FRGresult result;
        std::vector<GraphVertex::Ptr> src_nodes, tgt_nodes;
        Association A;
        g3reg::EllipsoidMatcher matcher(src_cloud, tgt_cloud);
        robot_utils::TicToc front_end_timer, timer;
        if (config.front_end == "gem") {
            A = std::move(matcher.matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
        } else if (config.front_end == "fpfh") {
            A = std::move(fpfh::matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
        } else if (config.front_end == "iss_fpfh") {
            A = std::move(iss_fpfh::matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
        } else if (config.front_end == "fcgf") {
            A = std::move(fcgf::matching(pair_info, src_nodes, tgt_nodes));
        } else if (config.front_end == "none") {
        }
        result.feature_time = front_end_timer.toc();

        if (config.back_end == "pagor") {
            pagor::solve(src_nodes, tgt_nodes, A, matcher, result);
        } else if (config.back_end == "ransac") {
            ransac::solve(src_nodes, tgt_nodes, A, result);
        } else if (config.back_end == "3dmac") {
            mac_reg::solve(src_nodes, tgt_nodes, A, result);
        } else {
            throw std::runtime_error("Unknown back end method");
        }

        result.total_time = timer.toc();
        return result;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr eigenToPCL(const Eigen::MatrixX3d &eigen_matrix) {
        // 创建一个新的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 设置点云的大小以匹配 Eigen 矩阵的行数
        cloud->points.resize(eigen_matrix.rows());

        // 转换所有点
        for (int i = 0; i < eigen_matrix.rows(); ++i) {
            pcl::PointXYZ &pt = cloud->points[i];
            pt.x = eigen_matrix(i, 0);
            pt.y = eigen_matrix(i, 1);
            pt.z = eigen_matrix(i, 2);
        }

        // 调整点云中点的数量
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        return cloud;
    }

    FRGresult SolveFromCorresp(const Eigen::MatrixX3d &src_corresp,
                               const Eigen::MatrixX3d &tgt_corresp,
                               const Eigen::MatrixX3d &src_cloud,
                               const Eigen::MatrixX3d &tgt_cloud,
                               const Config &config_custom) {

        assert(src_corresp.rows() == tgt_corresp.rows());

        config = config_custom;

        FRGresult result;
        std::vector<GraphVertex::Ptr> src_nodes, tgt_nodes;
        g3reg::EllipsoidMatcher matcher(eigenToPCL(src_cloud), eigenToPCL(tgt_cloud));
        robot_utils::TicToc front_end_timer, timer;

        int64_t num_corresp = src_corresp.rows();
        Association A = clique_solver::Association::Zero(num_corresp, 2);

        src_nodes.reserve(num_corresp);
        tgt_nodes.reserve(num_corresp);
        for (int i = 0; i < num_corresp; i++) {
            A(i, 0) = i;
            A(i, 1) = i;
            const Eigen::Vector3d &center = src_corresp.row(i);
            src_nodes.push_back(clique_solver::create_vertex(center, config.vertex_info));
            const Eigen::Vector3d &center_tgt = tgt_corresp.row(i);
            tgt_nodes.push_back(clique_solver::create_vertex(center_tgt, config.vertex_info));
        }
        result.feature_time = front_end_timer.toc();

        if (config.back_end == "pagor") {
            pagor::solve(src_nodes, tgt_nodes, A, matcher, result);
        } else if (config.back_end == "ransac") {
            ransac::solve(src_nodes, tgt_nodes, A, result);
        } else if (config.back_end == "3dmac") {
            mac_reg::solve(src_nodes, tgt_nodes, A, result);
        } else {
            throw std::runtime_error("Unknown back end method");
        }

        result.total_time = timer.toc();
        return result;
    }
}