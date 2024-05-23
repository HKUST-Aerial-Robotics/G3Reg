/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_KITTI_LOADER_H
#define SRC_KITTI_LOADER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/format.hpp>
#include "utils/config.h"
#include "dataset/kitti_utils.h"
#include "datasets/dataloader.h"

class KittiLoader : public DataLoader {

public:
    KittiLoader() : DataLoader() {
        LOG(INFO) << "Load Kitti Dataset.";
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string dataset_root, int seq, int i) {
        return kitti_utils::GetCloud(dataset_root, seq, i);
    }

    void LoadLiDARPoses(std::string dataset_root, int seq) {
        std::string pose_file = (boost::format("%s/%02d/poses.txt") % g3reg::config::dataset_root % seq).str();
        //    read kitti pose txt
        std::fstream f;
        f.open(pose_file, std::ios::in);
        if (!f.is_open()) {
            LOG(FATAL) << "Cannot open pose file: " << pose_file;
        }
        Eigen::Matrix4d Tr = GetTr(g3reg::config::dataset_root, seq); // lidar to camera
        std::string line;
        int num = -1;
        lidar_poses[seq] = std::map<int, Eigen::Matrix4d>();
        while (std::getline(f, line)) {
            num++;
            std::stringstream ss(line);
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 4; ++j) {
                    ss >> Twc(i, j);
                }
            }
            Eigen::Matrix4d Twl = Twc * Tr;
            lidar_poses[seq][num] = Twl;
        }
        f.close();
    }

    Eigen::Matrix4d GetTr(std::string dataset_root, int seq) {
        std::string calib_file = boost::str(boost::format("%s/%02d/calib.txt") % dataset_root % seq);
        std::fstream f;
        f.open(calib_file, std::ios::in);
        if (!f.is_open()) {
            std::cerr << "Cannot open calib file: " << calib_file << std::endl;
        }
        std::string line;
        Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            std::string tag;
            ss >> tag;
            if (tag == "Tr:") {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Tr(i, j);
                    }
                }
            }
        }
        return Tr;
    }

    void ReadSeqNum() {
        std::string dataset_dir = g3reg::config::dataset_root;
        std::map<int, int> frame_num_map;
        // traverse all sequences
        for (int seq = 0; seq < 11; ++seq) {
            std::string seq_dir = boost::str(boost::format("%s/%02d") % dataset_dir % seq);
            std::string velo_dir = boost::str(boost::format("%s/velodyne") % seq_dir);
            // count the number of bin in velo_dir
            int frame_num = 0;
            for (boost::filesystem::directory_iterator it(velo_dir);
                 it != boost::filesystem::directory_iterator(); ++it) {
                if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".bin") {
                    frame_num++;
                }
            }
            frame_num_map[seq] = frame_num;
        }
        LOG(INFO) << "Loading Kitti " << frame_num_map.size() << " sequences";
    }
};


#endif //SRC_KITTI_LOADER_H
