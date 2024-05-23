/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "utils/evaluation.h"
#include "front_end/graph_vertex.h"
#include "front_end/fpfh_utils.h"
#include "front_end/gem/ellipsoid.h"
#include "front_end/fcgf.h"
#include "utils/config.h"

using namespace clique_solver;
using namespace g3reg;

Eigen::Matrix3Xd Nodes2Matrix(const std::vector<GraphVertex::Ptr> &nodes) {
    Eigen::Matrix3Xd mat(3, nodes.size());
    for (int i = 0; i < nodes.size(); ++i) {
        mat.col(i) = nodes[i]->centroid;
    }
    return mat;
}

FeatureMetric FrontEndEval(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                           std::tuple<int, int, int> pair_info, Eigen::Matrix4d T_gt) {

    FeatureMetric result;

    std::vector<GraphVertex::Ptr> src_nodes, tgt_nodes;
    Association A;
    g3reg::EllipsoidMatcher matcher(src_cloud, tgt_cloud);
    robot_utils::TicToc timer;
    if (config::front_end == "gem") {
        A = std::move(matcher.matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
    } else if (config::front_end == "segregator") {
//        A = std::move(segregator::SemanticMatch(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
    } else if (config::front_end == "fpfh") {
        A = std::move(fpfh::matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
    } else if (config::front_end == "fcgf") {
        A = std::move(fcgf::matching(pair_info, src_nodes, tgt_nodes));
    } else {
        std::cout << "Wrong front end type!" << std::endl;
        exit(-1);
    }
    result.time = timer.toc();

    Eigen::Matrix3Xd src_mat = Nodes2Matrix(src_nodes);
    Eigen::Matrix3Xd tgt_mat = Nodes2Matrix(tgt_nodes);
    Eigen::Matrix3Xd src_mat_t = T_gt.block(0, 0, 3, 3) * src_mat + T_gt.block(0, 3, 3, 1).replicate(1, src_mat.cols());

    for (int i = 0; i < A.rows(); ++i) {
        if ((src_mat_t.col(A(i, 0)) - tgt_mat.col(A(i, 1))).norm() < config::tp_thresh) {
            result.tp++;
        }
    }
    if (A.rows() > 0)
        result.precision = result.tp / A.rows() * 100.0;
    else
        result.precision = 0;

    result.assoc_num = A.rows();

    return result;
}