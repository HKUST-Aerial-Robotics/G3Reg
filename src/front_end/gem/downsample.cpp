/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
//
#include "front_end/gem/downsample.h"

namespace pcl {

    pcl::PointCloud<pcl::PointXYZI>::Ptr toXYZI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, *cloud_i);
        return cloud_i;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr toXYZI(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_i->resize(cloud->size());
        for (int i = 0; i < cloud->size(); ++i) {
            cloud_i->points[i].x = cloud->points[i].x;
            cloud_i->points[i].y = cloud->points[i].y;
            cloud_i->points[i].z = cloud->points[i].z;
            cloud_i->points[i].intensity = cloud->points[i].label;
        }
        return cloud_i;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *cloud_i);
        return cloud_i;
    }
}