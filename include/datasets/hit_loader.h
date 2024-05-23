/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_HIT_MULTI_LOADER_H
#define SRC_HIT_MULTI_LOADER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/format.hpp>
#include "dataloader.h"
#include <robot_utils/lie_utils.h>

class HITLoader : public DataLoader {

public:
    HITLoader() : DataLoader() {
        LOG(INFO) << "Load HIT_MULTI Dataset.";
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string dataset_root, int seq, int i) {
        std::string file_name = boost::str(boost::format("%s/%02d/pcd/%d.pcd") % dataset_root % seq % i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) {
            LOG(ERROR) << "Couldn't read file " << file_name;
        }
        return cloud;
    }

    void LoadLiDARPoses(std::string dataset_root, int seq) {
        std::string pose_file = boost::str(boost::format("%s/%02d/poses.txt") % dataset_root % seq);
        std::ifstream file(pose_file);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file " + pose_file);
        }
        std::string line;
        int idx = 0;
        double x, y, z, qx, qy, qz, qw;
        lidar_poses[seq] = std::map<int, Eigen::Matrix4d>();
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            iss >> x, iss >> y, iss >> z, iss >> qx, iss >> qy, iss >> qz, iss >> qw;
            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            tf.block<3, 3>(0, 0) = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
            tf.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
            lidar_poses[seq][idx] = tf;
            idx++;
        }
    }

    Eigen::Matrix4d getLidarPose(std::string dataset_root, int seq, int frame_id) {
        if (lidar_poses.find(seq) == lidar_poses.end()) {
            LoadLiDARPoses(dataset_root, seq);
        } else if (lidar_poses[seq].find(frame_id) == lidar_poses[seq].end()) {
            LoadLiDARPoses(dataset_root, seq);
        } else {
            return lidar_poses[seq][frame_id];
        }
        return lidar_poses[seq][frame_id];
    }
};


#endif //SRC_HIT_MULTI_LOADER_H
