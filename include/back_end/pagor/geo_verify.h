/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_GEO_VERIFY_H
#define SRC_GEO_VERIFY_H

#include "front_end/gem/voxel.h"
#include "front_end/gem/downsample.h"

Eigen::Matrix4d GeometryVerify(typename pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                               typename pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                               const std::vector<Eigen::Matrix4d> &candidates);

std::pair<bool, Eigen::Matrix4d> GeometryVerify(const VoxelMap &voxel_map_src,
                                                const VoxelMap &voxel_map_tgt,
                                                const std::vector<Eigen::Matrix4d> &candidates);

std::pair<bool, Eigen::Matrix4d> PlaneVerify(typename pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                             typename pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                             const std::vector<Eigen::Matrix4d> &candidates);

#endif //SRC_GEO_VERIFY_H
