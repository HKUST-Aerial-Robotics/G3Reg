/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "tgs.hpp"
#include "utils/config.h"
#include <pcl/keypoints/iss_3d.h>

namespace pcl {

    pcl::PointCloud<pcl::PointXYZI>::Ptr toXYZI(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud);

    template<typename T>
    void issKeyPointExtration(boost::shared_ptr<pcl::PointCloud<T>> cloud, boost::shared_ptr<pcl::PointCloud<T>> ISS,
                              pcl::PointIndicesPtr ISS_Idx, double resolution) {
        double iss_salient_radius_ = 6 * resolution;
        double iss_non_max_radius_ = 4 * resolution;
        //double iss_non_max_radius_ = 2 * resolution;//for office
        //double iss_non_max_radius_ = 9 * resolution;//for railway
        double iss_gamma_21_(0.975);
        double iss_gamma_32_(0.975);
        double iss_min_neighbors_(4);
        int iss_threads_(1); //switch to the number of threads in your cpu for acceleration

        boost::shared_ptr<pcl::search::KdTree<T>> tree(new pcl::search::KdTree<T>());
        pcl::ISSKeypoint3D<T, T> iss_detector;

        iss_detector.setSearchMethod(tree);
        iss_detector.setSalientRadius(iss_salient_radius_);
        iss_detector.setNonMaxRadius(iss_non_max_radius_);
        iss_detector.setThreshold21(iss_gamma_21_);
        iss_detector.setThreshold32(iss_gamma_32_);
        iss_detector.setMinNeighbors(iss_min_neighbors_);
        iss_detector.setNumberOfThreads(iss_threads_);
        iss_detector.setInputCloud(cloud);
        iss_detector.compute(*ISS);
        ISS_Idx->indices = iss_detector.getKeypointsIndices()->indices;
        ISS_Idx->header = iss_detector.getKeypointsIndices()->header;
    }

    template<typename T>
    void voxelize(
            const boost::shared_ptr<pcl::PointCloud<T>> srcPtr, boost::shared_ptr<pcl::PointCloud<T>> dstPtr,
            double voxelSize) {
        static pcl::VoxelGrid<T> voxel_filter;
        voxel_filter.setInputCloud(srcPtr);
        voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        voxel_filter.filter(*dstPtr);
    }

    template<typename PointT>
    Eigen::MatrixXf PointCloud2Matrix(const pcl::PointCloud<PointT> &cloud) {
        Eigen::MatrixXf mat(cloud.size(), 3);
        for (int i = 0; i < cloud.size(); ++i) {
            mat(i, 0) = cloud.points[i].x;
            mat(i, 1) = cloud.points[i].y;
            mat(i, 2) = cloud.points[i].z;
        }
        return mat;
    }

    template<typename PointT>
    Eigen::MatrixXd PointCloud2Matrixd(const pcl::PointCloud<PointT> &cloud) {
        Eigen::MatrixXd mat(cloud.size(), 3);
        for (int i = 0; i < cloud.size(); ++i) {
            mat(i, 0) = cloud.points[i].x;
            mat(i, 1) = cloud.points[i].y;
            mat(i, 2) = cloud.points[i].z;
        }
        return mat;
    }

    template<typename PointT>
    void Matrix2PointCloud(const Eigen::MatrixXf &mat, pcl::PointCloud<PointT> &cloud) {
        cloud.clear();
        for (int i = 0; i < mat.rows(); ++i) {
            PointT point;
            point.x = mat(i, 0);
            point.y = mat(i, 1);
            point.z = mat(i, 2);
            cloud.push_back(point);
        }
    }

    template<typename PointT>
    void BoxFilter(pcl::PointCloud<PointT> &cloud,
                   pcl::PointCloud<PointT> &cloud_filtered, double max_range) {
        pcl::CropBox<PointT> boxFilter;
        boxFilter.setInputCloud(cloud.makeShared());
        boxFilter.setMin(Eigen::Vector4f(-max_range, -max_range, -max_range, 1.0));
        boxFilter.setMax(Eigen::Vector4f(max_range, max_range, max_range, 1.0));
        boxFilter.filter(cloud_filtered);
    }
}

namespace travel {
    template<typename PointT>
    void estimateGround(pcl::PointCloud<PointT> &cloud,
                        pcl::PointCloud<PointT> &ground,
                        pcl::PointCloud<PointT> &nonground,
                        double &time_taken) {
        travel::TravelGroundSeg<PointT> travel_ground_seg;
        travel_ground_seg.setParams(g3reg::config::max_range, g3reg::config::min_range);
        travel_ground_seg.estimateGround(cloud, ground, nonground, time_taken);
    }
}