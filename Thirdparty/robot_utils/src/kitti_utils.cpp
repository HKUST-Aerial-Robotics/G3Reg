/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "dataset/kitti_utils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "boost/format.hpp"
#include <iostream>
#include <fstream>
#include "dataset/kitti_definition.h"

namespace kitti_utils {

    std::string GetPCDPath(std::string dataset_root, int seq, int frame_id) {
        std::string filename = (boost::format("%s/%02d/velodyne/%06d.bin") % dataset_root % seq % frame_id).str();
        return filename;
    }

    std::string GetLabelPath(std::string dataset_root, int seq, int frame_id, std::string label_dir) {
        std::string filename = (boost::format("%s/%02d/%s/%06d.label") % dataset_root % seq % label_dir % frame_id).str();
        return filename;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string dataset_root, int seq, int frame_id) {
        std::string filename = GetPCDPath(dataset_root, seq, frame_id);
        return ReadCloudXYZ(filename);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudI(std::string dataset_root, int seq, int frame_id) {
        std::string filename = GetPCDPath(dataset_root, seq, frame_id);
        return ReadCloudBin(filename);
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr GetCloud(std::string dataset_root, int seq, int frame_id, std::string label_dir) {
        const std::string label_file_name = GetLabelPath(dataset_root, seq, frame_id, label_dir);
        std::ifstream flabel(label_file_name.c_str(), std::ios::binary);
        if (!flabel) {
            std::cerr << "[Error][KittiParser::ReadSemColorCloud] Point Cloud Label File '" << label_file_name
                      << "' is not found!" << std::endl;
            return 0;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr sem_cloud = GetCloudI(dataset_root, seq, frame_id);
        int cloud_size = sem_cloud->size();
        std::vector<uint32_t> label_buffer(cloud_size);
        flabel.read(reinterpret_cast<char *>(&label_buffer[0]), cloud_size * sizeof(uint32_t));
        flabel.close();

        pcl::PointCloud<pcl::PointXYZL>::Ptr sem_cloud_l(new pcl::PointCloud<pcl::PointXYZL>());
        pcl::copyPointCloud(*sem_cloud, *sem_cloud_l);
        for (uint32_t i = 0; i < cloud_size; i += 4) {
            const uint16_t sem_label = label_buffer[i] & 0xFFFF;
            //const uint16_t inst_label = label_buffer[i / 4] >> 16;
            pcl::PointXYZL&point = sem_cloud_l->points[i];
            point.label = sem_label;
        }
        return sem_cloud_l;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ReadCloudBin(std::string bin_path){
        // N*4 float
        std::ifstream fcloud(bin_path.c_str(), std::ios::binary);
        if (!fcloud) {
            std::cerr << "[Error] Point Cloud File '" << bin_path << "' is not found!" << std::endl;
            return nullptr;
        }

        // get length of input file:
        fcloud.seekg(0, fcloud.end);
        int cloud_length = (int) fcloud.tellg();
        int cloud_size = cloud_length / sizeof(float);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->reserve(cloud_size / 4);
        std::vector<float> point_buffer(cloud_size);
        fcloud.seekg(0, fcloud.beg);
        fcloud.read(reinterpret_cast<char *>(&point_buffer[0]), cloud_size * sizeof(float));
        fcloud.close();
        pcl::PointXYZI point;
        for (uint32_t i = 0; i < point_buffer.size(); i += 4) {
            point.x = point_buffer[i];
            point.y = point_buffer[i + 1];
            point.z = point_buffer[i + 2];
            point.intensity = point_buffer[i + 3];
            cloud->push_back(point);
        }
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ReadCloudXYZ(std::string bin_path){
        // N*4 float
        std::ifstream fcloud(bin_path.c_str(), std::ios::binary);
        if (!fcloud) {
            std::cerr << "[Error] Point Cloud File '" << bin_path << "' is not found!" << std::endl;
            return nullptr;
        }

        // get length of input file:
        fcloud.seekg(0, fcloud.end);
        int cloud_length = (int) fcloud.tellg();
        int cloud_size = cloud_length / sizeof(float);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->resize(cloud_size / 4);
        std::vector<float> point_buffer(cloud_size);
        fcloud.seekg(0, fcloud.beg);
        fcloud.read(reinterpret_cast<char *>(&point_buffer[0]), cloud_size * sizeof(float));
        fcloud.close();
        for (uint32_t i = 0; i < point_buffer.size(); i += 4) {
            pcl::PointXYZ& point = cloud->points[i/4];
            point.x = point_buffer[i];
            point.y = point_buffer[i + 1];
            point.z = point_buffer[i + 2];
        }
        return cloud;
    }

    void SaveSemLabel(std::string dataset_root, int seq, int frame_id, const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud
            , std::string label_dir) {
        const std::string label_file_name = GetLabelPath(dataset_root, seq, frame_id, label_dir);
        std::ofstream flabel(label_file_name.c_str(), std::ios::binary);
        if (!flabel) {
            std::cerr << "[Error][KittiParser::ReadSemColorCloud] Point Cloud Label File '" << label_file_name
                      << "' is not found!" << std::endl;
            return;
        }

        std::vector<uint32_t> label_buffer(cloud->size());
        for (uint32_t i = 0; i < cloud->size(); ++i) {
            label_buffer[i] = cloud->at(i).label;
        }

        flabel.write(reinterpret_cast<char *>(&label_buffer[0]), cloud->size() * sizeof(uint32_t));
        flabel.close();
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

    Eigen::Matrix4d getLidarPose(std::string dataset_root, int seq, int frame_id) {
        std::string pose_file = (boost::format("%s/%02d/poses.txt") % dataset_root % seq).str();
        //    read kitti pose txt
        std::fstream f;
        f.open(pose_file, std::ios::in);
        if (!f.is_open()) {
            std::cerr << "Cannot open pose file: " << pose_file << std::endl;
        }
        Eigen::Matrix4d Tr = GetTr(dataset_root, seq);
        std::string line;
        int num = 0;
        while (std::getline(f, line)) {
            if (num == frame_id) {
                std::stringstream ss(line);
                Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Twc(i, j);
                    }
                }
                Eigen::Matrix4d Twl = Twc * Tr;
                return Twl;
            }
            num++;
        }
        return Eigen::Matrix4d::Identity();
    }

    int getFramesNum(std::string dataset_root, int seq) {
        std::string bin_dir = (boost::format("%s/%02d/velodyne") % dataset_root % seq).str();
        int num = 0;
        // count the number of bin files in the directory
        for (boost::filesystem::directory_iterator it(bin_dir); it != boost::filesystem::directory_iterator(); ++it) {
            if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".bin") {
                num++;
            }
        }
        return num;
    }
    
    bool isGround(const uint16_t sem_id) {
        switch (sem_id) {
            case ROAD_ID:
            case PARKING_ID:
            case SIDEWALK_ID:
            case OTHER_GROUND_ID:
            case LANE_MARKING_ID:
                return true;
            default:
                return false;
        }
    }

    bool isPillar(const uint16_t sem_id) {
        switch (sem_id) {
            case POLE_ID:
            case TRUNK_ID:
                return true;
            default:
                return false;
        }
    }

    bool isPlane(const uint16_t sem_id) {
        switch (sem_id) {
            case BUILDING_ID:
            case FENCE_ID:
            case OTHER_STRUCT_ID:
                return true;
            default:
                return false;
        }
    }
}
