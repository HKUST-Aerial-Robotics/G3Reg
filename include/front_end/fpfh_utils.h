/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
//

#ifndef SRC_FPFH_UTILS_H
#define SRC_FPFH_UTILS_H

#include <fpfh_matcher.h>
#include "utils/config.h"
#include "utils/evaluation.h"

namespace fpfh{

    void Match(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                   std::vector<std::pair<int, int>> &corres);

    clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                        std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes);

    FeatureMetric Evaluate(const int &seq, const int &src_id, const int &tgt_id, const Eigen::Matrix4d& T_gt);
}

namespace iss_fpfh{
	clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
	                                    std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes);
}

#endif //SRC_FPFH_UTILS_H
