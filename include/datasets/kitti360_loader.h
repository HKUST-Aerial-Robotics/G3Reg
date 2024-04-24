/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_KITTI360_LOADER_H
#define SRC_KITTI360_LOADER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/format.hpp>
#include "dataloader.h"
#include <robot_utils/lie_utils.h>

class KITTI360Loader: public DataLoader {
private:
    Eigen::Matrix4d calib_lidar_to_pose;

public:
    KITTI360Loader(): DataLoader() {
        LOG(INFO) << "Load KITTI360 Dataset.";
        calib_lidar_to_pose = getLidarToIMU(config::dataset_root);
    }

    Eigen::Matrix4d getEgoMotion(int seq, int frame_id) {
		Eigen::Matrix4d tf = getLidarPose(config::dataset_root, seq, frame_id);
        if (lidar_poses[seq].find(frame_id) != lidar_poses[seq].end() && lidar_poses[seq].find(frame_id - 1) != lidar_poses[seq].end()) {
            return lidar_poses[seq][frame_id].inverse() * lidar_poses[seq][frame_id - 1];
        } else if (lidar_poses[seq].find(frame_id) != lidar_poses[seq].end() && lidar_poses[seq].find(frame_id + 1) != lidar_poses[seq].end()) {
            return lidar_poses[seq][frame_id + 1].inverse() * lidar_poses[seq][frame_id];
        } else {
            return Eigen::Matrix4d::Identity();
        }
    }

    void deskew(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3d rho, Eigen::Vector3d t){
#pragma omp parallel for
        for (int i = 0; i < cloud->size(); ++i) {
            double vx, vy, vz, s, rx, ry, rz, tx, ty, tz, theta, kx, ky, kz, ct, st, kv;
            pcl::PointXYZ& pt = cloud->points[i];
            vx = pt.x, vy = pt.y, vz = pt.z;
            s = 0.5 * std::atan2(vy, vx) / M_PI;
            rx = s * rho[0], ry = s * rho[1], rz = s * rho[2];
            tx = s * t[0], ty = s * t[1], tz = s * t[2];
            theta = std::sqrt(rx * rx + ry * ry + rz * rz);
            if (theta > 1e-10) {
                kx = rx / theta, ky = ry / theta, kz = rz / theta;
                ct = std::cos(theta);
                st = std::sin(theta);
                kv = kx * vx + ky * vy + kz * vz;
                pt.x = vx * ct + (ky * vz - kz * vy) * st + kx * kv * (1 - ct) + tx;
                pt.y = vy * ct + (kz * vx - kx * vz) * st + ky * kv * (1 - ct) + ty;
                pt.z = vz * ct + (kx * vy - ky * vx) * st + kz * kv * (1 - ct) + tz;
            } else {
                pt.x = vx + tx, pt.y = vy + ty, pt.z = vz + tz;
            }
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string dataset_root, int seq, int i) {
        std::string file_name = boost::str(boost::format("%s/data_3d_raw/2013_05_28_drive_%04d_sync/velodyne_points/data/%010d.bin") % dataset_root % seq % i);
        robot_utils::TicToc t;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = kitti_utils::ReadCloudXYZ(file_name);
        Eigen::Matrix4d ego_motion = getEgoMotion(seq, i);
        if ((ego_motion- Eigen::Matrix4d::Identity()).norm() > 0.1){
            Eigen::Matrix3d R = ego_motion.block<3,3>(0,0);
            Eigen::Vector3d rho = robot_utils::R2so3(R);
            Eigen::Vector3d t = ego_motion.block<3,1>(0,3);
            deskew(cloud, rho, t);
        }
        return cloud;
    }
	
	void LoadLiDARPoses(std::string dataset_root, int seq){
		std::string pose_file = boost::str(boost::format("%s/data_poses/2013_05_28_drive_%04d_sync/poses.txt") % dataset_root % seq);
		std::ifstream file(pose_file);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file " + pose_file);
		}
		std::string line;
		int idx;
		lidar_poses[seq] = std::map<int, Eigen::Matrix4d>();
		while (std::getline(file, line)) {
			std::istringstream iss(line);
			iss >> idx;
			Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
			for (int i = 0; i < 3; i++) {
				iss >> tf(i, 0) >> tf(i, 1) >> tf(i, 2) >> tf(i, 3);
			}
			tf = tf * calib_lidar_to_pose;
			lidar_poses[seq][idx] = tf;
		}
	}
	
	Eigen::Matrix4d getLidarPose(std::string dataset_root, int seq, int frame_id) {
		if (lidar_poses.find(seq) == lidar_poses.end()){
			LoadLiDARPoses(dataset_root, seq);
		} else if (lidar_poses[seq].find(frame_id) == lidar_poses[seq].end()){
			LoadLiDARPoses(dataset_root, seq);
		} else {
			return lidar_poses[seq][frame_id];
		}
		return lidar_poses[seq][frame_id];
	}

    Eigen::Matrix4d getLidarToIMU(std::string dataset_root){
        std::string calib_cam_to_pose_file = boost::str(boost::format("%s/calibration/calib_cam_to_pose.txt") % dataset_root);
        std::string calib_cam_to_velo_file = boost::str(boost::format("%s/calibration/calib_cam_to_velo.txt") % dataset_root);
        Eigen::Matrix4d calib_cam_to_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d calib_cam_to_velo = Eigen::Matrix4d::Identity();
        std::ifstream file(calib_cam_to_pose_file);
        if (!file.is_open()) {
            std::cerr << "[Error] Point Cloud File '" << calib_cam_to_pose_file << "' is not found!" << std::endl;
            throw std::runtime_error("Could not open file");
        }
        std::string line;
        std::getline(file, line);
        line = line.substr(9);
        std::istringstream iss(line);
        for (int i = 0; i < 3; i++) {
            iss >> calib_cam_to_pose(i, 0) >> calib_cam_to_pose(i, 1) >> calib_cam_to_pose(i, 2) >> calib_cam_to_pose(i, 3);
        }
        file.close();
        file.open(calib_cam_to_velo_file);
        if (!file.is_open()) {
            std::cerr << "[Error] Point Cloud File '" << calib_cam_to_velo_file << "' is not found!" << std::endl;
            throw std::runtime_error("Could not open file");
        }
        std::getline(file, line);
        iss = std::istringstream(line);
        for (int i = 0; i < 3; i++) {
            iss >> calib_cam_to_velo(i, 0) >> calib_cam_to_velo(i, 1) >> calib_cam_to_velo(i, 2) >> calib_cam_to_velo(i, 3);
        }
        file.close();
        Eigen::Matrix4d calib_lidar_to_pose = calib_cam_to_pose * calib_cam_to_velo.inverse();
        return calib_lidar_to_pose;
    }

};


#endif //SRC_KITTI360_LOADER_H
