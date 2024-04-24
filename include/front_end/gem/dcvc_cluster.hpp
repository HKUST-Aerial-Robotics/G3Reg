/*****************************************************************
 *
 * Copyright (c) 2022, Nanyang Technological University, Singapore
 *
 * Authors: Pengyu Yin
 * Contact: pengyu001@e.ntu.edu.sg
 *
 * DCVC Code based on: T-LOAM: Truncated Least Squares LiDAR-Only Odometry and Mapping in Real Time
 * link: https://github.com/zpw6106/tloam
 * 
 * fpfh feature extraction/G-TRIM code based on: Teaser++ and Quatro
 * link: https://github.com/MIT-SPARK/TEASER-plusplus
 * link: https://github.com/url-kaist/Quatro
 * 
 ****************************************************************/

#ifndef CLUSTER_MANAGER_H
#define CLUSTER_MANAGER_H

#include <unordered_map>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <yaml-cpp/yaml.h>

template<typename PointT>
class DCVCCluster {
public:
    struct DCVCParam {
        //KITTI
        double startR = 0.35;
        double deltaR = 0.0004;
        double deltaP = 1.2;
        double deltaA = 1.2;
        int minSeg = 80;
        double max_range = 120;
        double min_range = 0.5;
    };
    using PointCloudPtr = boost::shared_ptr<pcl::PointCloud<PointT>>;

private:

    DCVCParam params_;

    // DBScan/Hierarchical DBScan related
    PointCloudPtr cloud_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
    std::vector<PointCloudPtr> clusters_;

    // curved voxelization related
    double minPitch{0.0};
    double maxPitch{0.0};
    double minPolar{5.0};
    double maxPolar{5.0};
    int width{0};
    int height{0};
    // int minSeg{0};
    int polarNum{0};
    std::vector<double> polarBounds{};
    std::vector<Eigen::Vector3d> polarCor;
    std::unordered_map<int, std::vector<int>> voxelMap{};

    std::vector<Eigen::Matrix3d> covariances_src_;
    std::vector<Eigen::Matrix3d> covariances_tgt_;

public:

    template<typename T>
    T get(const YAML::Node &node, const std::string &father_key, const std::string &key, const T &default_value) {
        if (!node[father_key] || !node[father_key][key]) {
//        std::cout << "Key " << father_key << "/" << key << " not found, using default value: " << default_value << std::endl;
            return default_value;
        }
        T value = node[father_key][key].as<T>();
//    std::cout << "Key " << father_key << "/" << key << " found, using value: " << value << std::endl;
        return value;
    }

    PointCloudPtr sem_cloud_in;

    DCVCCluster(DCVCParam &params) {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        params_ = params;
        clusters_.clear();
    }

    DCVCCluster(std::string &config_path){

        YAML::Node config_node = YAML::LoadFile(config_path);
        params_.startR = get(config_node, "dcvc", "startR", 0.35);
        params_.deltaR = get(config_node, "dcvc", "deltaR", 0.0004);
        params_.deltaP = get(config_node, "dcvc", "deltaP", 1.2);
        params_.deltaA = get(config_node, "dcvc", "deltaA", 1.2);
        params_.max_range = get(config_node, "dcvc", "max_range", 120.0);
        params_.min_range = get(config_node, "dcvc", "min_range", 0.5);
        params_.minSeg = get(config_node, "dcvc", "min_cluster_size", 20);

        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        clusters_.clear();
    }

    /** \brief Empty destructor */
    ~DCVCCluster() {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // following code for dynamic voxel segmentation
    bool segmentPointCloud(PointCloudPtr input_sem_cloud, std::vector<PointCloudPtr> &clusters) {
        if (input_sem_cloud->size() == 0) {
            return false;
        }
        pcl::copyPointCloud(*input_sem_cloud, *cloud_);
        // step1, scan->polar coordinate
        convert2polar();

        // step 2, create hash table
        createHashTable();

        // step 3, DCVC segmentation
        std::vector<int> labelInfo{};
        if (!DCVC(labelInfo)) {
            //ROS_ERROR("DCVC algorithm segmentation failure");
            return false;
        }

        // step 4, store segmentation results to clusters_
        labelAnalysis(labelInfo);

        // step 5, store clusters_ to clusters
        clusters = clusters_;
        return true;
    }

    void convert2polar() {
        if (cloud_->points.size() == 0) {
            std::cerr << "Point cloud empty in converting cartesian to polar!" << std::endl;
        }

        // culculate yaw angle(rad)
        double rad2deg = 180.0 / M_PI;
        auto azimuthCal = [&](double x, double y) -> double {
            auto angle = static_cast<double>(std::atan2(y, x));
            return angle > 0.0 ? angle * rad2deg : (angle + 2 * M_PI) * rad2deg;
        };

        size_t totalSize = cloud_->points.size();
        polarCor.resize(totalSize);

        Eigen::Vector3d cur = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < totalSize; ++i) {
            // polar pitch azimuth
            Eigen::Vector3d rpa = Eigen::Vector3d::Zero();
            cur(0) = cloud_->points[i].x;
            cur(1) = cloud_->points[i].y;
            cur(2) = cloud_->points[i].z;
            rpa.x() = cur.norm(); // range
            rpa.y() = std::asin(cur.z() / rpa.x()) * rad2deg; // pitch
            rpa.z() = azimuthCal(cur.x(), cur.y()); // azimuth

            // filter out points with range < 0.5m or > 120m
            if (rpa.x() >= params_.max_range || rpa.x() <= params_.min_range)
                continue;

            // record min/max pitch and range
            minPitch = rpa.y() < minPitch ? rpa.y() : minPitch;
            maxPitch = rpa.y() > maxPitch ? rpa.y() : maxPitch;
            minPolar = rpa.x() < minPolar ? rpa.x() : minPolar;
            maxPolar = rpa.x() > maxPolar ? rpa.x() : maxPolar;

            polarCor[i] = rpa;
        }

        polarCor.shrink_to_fit(); // release memory

        polarNum = 0;
        polarBounds.clear();
        width = static_cast<int>(std::round(360.0 / params_.deltaA) + 1);
        height = static_cast<int>((maxPitch - minPitch) / params_.deltaP);
        double range = minPolar;
        int step = 1;
        while (range <= maxPolar) {
            range += (params_.startR - step * params_.deltaR);
            polarBounds.emplace_back(range);
            polarNum++, step++;
        }
    }

    void createHashTable() {
        size_t totalSize = polarCor.size();

        Eigen::Vector3d cur = Eigen::Vector3d::Zero();
        int polarIndex, pitchIndex, azimuthIndex, voxelIndex;
        voxelMap.reserve(totalSize);

        for (size_t item = 0; item < totalSize; ++item) {
            cur = polarCor[item];
            polarIndex = getPolarIndex(cur.x());
            pitchIndex = static_cast<int>(std::round((cur.y() - minPitch) / params_.deltaP));
            azimuthIndex = static_cast<int>(std::round(cur.z() / params_.deltaA));

            voxelIndex = (azimuthIndex * (polarNum + 1) + polarIndex) + pitchIndex * (polarNum + 1) * (width + 1);

            auto iter = voxelMap.find(voxelIndex);
            if (iter != voxelMap.end()) {
                //iter->second.index.emplace_back(item);
                iter->second.emplace_back(item);
            } else {
                std::vector<int> index{};
                index.emplace_back(item);
                voxelMap.insert(std::make_pair(voxelIndex, index));
            }
        }
    }

    /**
     * @brief get the index value in the polar radial direction
     * @param radius, polar diameter
     * @return polar diameter index
     */
    int getPolarIndex(double &radius) {

        for (auto r = 0; r < polarNum; ++r) {
            if (radius < polarBounds[r])
                return r;
        }
        return polarNum - 1;
    }

    /**
     * @brief the Dynamic Curved-Voxle Clustering algoithm for fast and precise point cloud segmentaiton
     * @param label_info, output the category information of each point
     * @return true if success otherwise false
     */
    bool DCVC(std::vector<int> &label_info) {

        int labelCount = 0;
        size_t totalSize = polarCor.size(); // total points in the cloud
        if (totalSize <= 0) {
            std::cerr << "points in the cloud not enough to complete the DCVC algorithm" << std::endl;
            return false;
        }

        label_info.resize(totalSize, -1);
        Eigen::Vector3d cur = Eigen::Vector3d::Zero();
        int polar_index, pitch_index, azimuth_index, voxel_index, currInfo, neighInfo;

        for (size_t i = 0; i < totalSize; ++i) {
            if (label_info[i] != -1)
                continue;
            cur = polarCor[i];

            // get the voxel index of the current point
            polar_index = getPolarIndex(cur.x());
            pitch_index = static_cast<int>(std::round((cur.y() - minPitch) / params_.deltaP));
            azimuth_index = static_cast<int>(std::round(cur.z() / params_.deltaA));
            voxel_index = (azimuth_index * (polarNum + 1) + polar_index) + pitch_index * (polarNum + 1) * (width + 1);
            auto iter_find = voxelMap.find(voxel_index);

            std::vector<int> neighbors;
            if (iter_find != voxelMap.end()) {
                std::vector<int> KNN{}; // find adjacent voxels 27 = 3 * 3 * 3
                searchKNN(polar_index, pitch_index, azimuth_index, KNN);

                for (auto &k: KNN) {
                    iter_find = voxelMap.find(k);

                    if (iter_find != voxelMap.end()) {
                        neighbors.reserve(iter_find->second.size());
                        for (auto &id: iter_find->second) {
                            neighbors.emplace_back(id); // get the point index in the adjacent voxels
                        }
                    }
                }
            }

            neighbors.swap(neighbors); // release memory

            if (!neighbors.empty()) {
                for (auto &id: neighbors) {
                    currInfo = label_info[i];       // current label index
                    neighInfo = label_info[id];     // voxel label index
                    if (currInfo != -1 && neighInfo != -1 && currInfo != neighInfo) {
                        for (auto &seg: label_info) {
                            if (seg == currInfo)
                                seg = neighInfo; // merge the two categories. Current to neighbor
                        }
                    } else if (neighInfo != -1) {
                        label_info[i] = neighInfo;
                    } else if (currInfo != -1) {
                        label_info[id] = currInfo;
                    } else {
                        continue;
                    }
                }
            }

            // If there is no category information yet, then create a new label information
            if (label_info[i] == -1) {
                labelCount++;
                label_info[i] = labelCount;
                for (auto &id: neighbors) {
                    label_info[id] = labelCount;
                }
            }
        }

        // free memory
        std::vector<Eigen::Vector3d>().swap(polarCor);

        return true;
    }

    /**
     * @brief search for neighboring voxels
     * @param polar_index, polar diameter index
     * @param pitch_index, pitch angular index
     * @param azimuth_index, azimuth angular index
     * @param out_neighIndex, output adjacent voxel index set
     * @return void
     */
    void searchKNN(int &polar_index, int &pitch_index, int &azimuth_index, std::vector<int> &out_neighIndex) const {

        for (auto z = pitch_index - 1; z <= pitch_index + 1; ++z) {
            if (z < 0 || z > height)
                continue;
            for (int y = polar_index - 1; y <= polar_index + 1; ++y) {
                if (y < 0 || y > polarNum)
                    continue;
                for (int x = azimuth_index - 1; x <= azimuth_index + 1; ++x) {
                    int ax = x;
                    if (ax < 0)
                        ax = width - 1;
                    if (ax >= width)
                        ax = width;
                    out_neighIndex.emplace_back((ax * (polarNum + 1) + y) + z * (polarNum + 1) * (width + 1));
                }
            }
        }
    }

    /**
     * @brief delete clusters with fewer points, store clusters into a vector of point clouds
     * @param label_info, input category information
     * @return void
     */
    void labelAnalysis(std::vector<int> &label_info) {

        std::unordered_map<int, std::vector<int>> label2segIndex;
        size_t totalSize = label_info.size();
        for (size_t i = 0; i < totalSize; ++i) {
            // zero initialization for unordered_map
            label2segIndex[label_info[i]].emplace_back(i);
        }

        for (auto &it: label2segIndex) {
            if (it.second.size() >= params_.minSeg) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (auto &idx: it.second) {
                    // cur_cloud->points.emplace_back(cloud_->points[idx]);
                    cur_cloud->points.push_back(cloud_->points[idx]);
                }
                clusters_.push_back(cur_cloud);
            }
        }
        // free memory
        std::unordered_map<int, std::vector<int>>().swap(label2segIndex);
    }

    std::vector<Eigen::Matrix3d> getSrcCovMat() {
        return covariances_src_;
    }

    std::vector<Eigen::Matrix3d> getTgtCovMat() {
        return covariances_tgt_;
    }

};

#endif