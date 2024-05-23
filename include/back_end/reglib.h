/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_REGLIB_H
#define SRC_REGLIB_H

#include "utils/config.h"
#include "utils/evaluation.h"
#include <pcl/point_cloud.h>

namespace g3reg {
    FRGresult GlobalRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_cloud,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt_cloud,
                                 std::tuple<int, int, int> pair_info = std::make_tuple(0, 0, 0));

    FRGresult SolveFromCorresp(const Eigen::MatrixX3d &src_corresp,
                               const Eigen::MatrixX3d &tgt_corresp,
                               const Eigen::MatrixX3d &src_cloud,
                               const Eigen::MatrixX3d &tgt_cloud,
                               const Config &config_custom = config);
}


#endif //SRC_REGLIB_H
