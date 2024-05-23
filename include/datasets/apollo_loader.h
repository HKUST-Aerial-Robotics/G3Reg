/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_APOLLO_LOADER_H
#define SRC_APOLLO_LOADER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/format.hpp>
#include "utils/config.h"
#include "datasets/dataloader.h"

class ApolloData {
public:
    std::vector<std::string> apollo_sessions = {
            "MapData/HighWay237/2018-10-05/",
            "MapData/SunnyvaleBigloop/Caspian_and_Geneva/2017-12-13/",
            "MapData/SunnyvaleBigloop/Borrgas/2017-12-13/",
            "MapData/SunnyvaleBigloop/Java/2017-12-13/",
            "MapData/SunnyvaleBigloop/Mathilda_Moffet/2017-12-28/",
            "MapData/SunnyvaleBigloop/Crossman/2017-12-13/",
            "MapData/SunnyvaleBigloop/Mathilda_Carribean/2017-12-14/",
            "MapData/SunnyvaleBigloop/Bordeaux/2017-12-13/",
            "MapData/MathildaAVE/2018-09-25/",
            "MapData/SanJoseDowntown/2018-10-02/",
            "MapData/BaylandsToSeafood/2018-09-26/",
            "MapData/ColumbiaPark/2018-09-21/2/",
            "MapData/ColumbiaPark/2018-09-21/4/",
            "MapData/ColumbiaPark/2018-09-21/1/",
            "MapData/ColumbiaPark/2018-09-21/3/",
            "TrainData/HighWay237/2018-10-12/",
            "TrainData/MathildaAVE/2018-10-04/",
            "TrainData/SanJoseDowntown/2018-10-11/",
            "TrainData/BaylandsToSeafood/2018-10-05/",
            "TrainData/ColumbiaPark/2018-10-03/",
            "TestData/HighWay237/2018-10-12/", // 20
            "TestData/SunnyvaleBigloop/2018-10-03/", // 21
            "TestData/MathildaAVE/2018-10-12/", // 22
            "TestData/SanJoseDowntown/2018-10-11/2/", // 23
            "TestData/SanJoseDowntown/2018-10-11/1/", // 24
            "TestData/BaylandsToSeafood/2018-10-12/", // 25
            "TestData/ColumbiaPark/2018-10-11/" // 26
    };
};

class ApolloLoader : public DataLoader {

public:
    typedef std::shared_ptr<ApolloLoader> Ptr;
    ApolloData apollo_data;

public:

    ApolloLoader() : DataLoader() {
        LOG(INFO) << "Load Apollo Dataset.";
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
        std::string dir_name = apollo_data.apollo_sessions[seq];
        std::string file_name = boost::str(boost::format("%s/%spcds/%d.pcd") % dataset_root % dir_name % i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        return cloud;
    }

    void LoadLiDARPoses(std::string dataset_root, int seq) {
        std::string dir_name = apollo_data.apollo_sessions[seq];
        std::string file_name = boost::str(boost::format("%s/%sposes/gt_poses.txt") % dataset_root % dir_name);
        std::ifstream file(file_name);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file");
        }
        std::fstream f;
        f.open(file_name, std::ios::in);
        std::string line;
        lidar_poses[seq] = std::map<int, Eigen::Matrix4d>();
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            int num;
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            ss >> num >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Eigen::Matrix4d Twl = Eigen::Matrix4d::Identity();
            Eigen::Quaterniond q(qw, qx, qy, qz);
            Eigen::Matrix3d R = q.toRotationMatrix();
            Twl.block<3, 3>(0, 0) = R;
            Twl(0, 3) = tx;
            Twl(1, 3) = ty;
            Twl(2, 3) = tz;
            lidar_poses[seq][num] = Twl;
        }
        f.close();
    }
};


#endif //SRC_APOLLO_LOADER_H
