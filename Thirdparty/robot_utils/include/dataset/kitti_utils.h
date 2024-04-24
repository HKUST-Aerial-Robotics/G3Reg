/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#pragma once
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kitti_utils {

    pcl::PointCloud<pcl::PointXYZI>::Ptr ReadCloudBin(std::string bin_path);
    
    std::string GetPCDPath(std::string dataset_root, int seq, int frame_id);

    std::string GetLabelPath(std::string dataset_root, int seq, int frame_id, std::string label_dir = "labels");

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string dataset_root, int seq, int frame_id);

    pcl::PointCloud<pcl::PointXYZL>::Ptr GetCloud(std::string dataset_root, int seq, int frame_id, std::string label_dir);

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudI(std::string dataset_root, int seq, int frame_id);

    void SaveSemLabel(std::string dataset_root, int seq, int frame_id,
                      const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, std::string label_dir = "labels");

    pcl::PointCloud<pcl::PointXYZ>::Ptr ReadCloudXYZ(std::string bin_path);

    Eigen::Matrix4d GetTr(std::string dataset_root, int seq);

    Eigen::Matrix4d getLidarPose(std::string dataset_root, int seq, int frame_id);

    int getFramesNum(std::string dataset_root, int seq);

    bool isGround(const uint16_t sem_id);

    bool isPillar(const uint16_t sem_id);

    bool isPlane(const uint16_t sem_id);
}