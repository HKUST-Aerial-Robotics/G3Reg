/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_CLUSTERING_H
#define SRC_CLUSTERING_H

#include "utils/config.h"
#include "aos.hpp"
#include "dcvc_cluster.hpp"
#include "downsample.h"
#include "gemodel.h"

namespace DCVC{
    template<typename PointT>
    void Cluster(boost::shared_ptr<pcl::PointCloud<PointT>> cloud,
                        std::vector<g3reg::ClusterFeature::Ptr>& clusters,
                        bool remove_ground = true) {
        double tSrc;
        pcl::PointCloud<PointT> srcGround;
        boost::shared_ptr<pcl::PointCloud<PointT>> ptrSrcNonground(new pcl::PointCloud<PointT>);
        if (remove_ground){
            travel::estimateGround(*cloud, srcGround, *ptrSrcNonground, tSrc);
        } else
            ptrSrcNonground = cloud;
        DCVCCluster<PointT> dcvc(config::dcvc_file);
        std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>> clusters_pcl;
        dcvc.segmentPointCloud(ptrSrcNonground, clusters_pcl);
        clusters.clear();
        for (auto &cluster_pcl: clusters_pcl) {
            g3reg::ClusterFeature::Ptr cluster_feature(new g3reg::ClusterFeature(cluster_pcl));
            clusters.push_back(cluster_feature);
        }
    }
}

namespace travel{
    template<typename PointT>
    void Cluster(boost::shared_ptr<pcl::PointCloud<PointT>> cloud_ptr, std::vector<g3reg::ClusterFeature::Ptr>& clusters, bool remove_ground = true) {
        double tSrc;
        pcl::PointCloud<PointT> srcGround;
        boost::shared_ptr<pcl::PointCloud<PointT>> ptrSrcNonground(new pcl::PointCloud<PointT>);
        if (remove_ground){
            travel::estimateGround(*cloud_ptr, srcGround, *ptrSrcNonground, tSrc);
        } else{
            ptrSrcNonground = cloud_ptr;
        }
        travel::ObjectCluster<PointT> travel_object_seg(config::travel_file);
        std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>> clusters_pcl;
        travel_object_seg.segmentObjects(ptrSrcNonground, clusters_pcl);
        clusters.clear();
        for (auto &cluster_pcl: clusters_pcl) {
            g3reg::ClusterFeature::Ptr cluster_feature(new g3reg::ClusterFeature(cluster_pcl));
            clusters.push_back(cluster_feature);
        }
    }
}

namespace pcl{
    template<typename PointT>
    void Cluster(boost::shared_ptr<pcl::PointCloud<PointT>> cloud,
                        std::vector<g3reg::ClusterFeature::Ptr>& clusters,
                        bool remove_ground = true) {

        pcl::search::KdTree<PointT> kdtree;
        kdtree.setInputCloud(cloud);

        // 在XY空间上进行聚类
        pcl::EuclideanClusterExtraction<PointT> cluster;
        cluster.setClusterTolerance(1.0);
        cluster.setMinClusterSize(config::min_cluster_size);
        cluster.setSearchMethod(&kdtree);
        cluster.setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_res;
        cluster.extract(cluster_res);

        clusters.clear();
        for (int i = 0; i < cluster_res.size(); ++i) {
            boost::shared_ptr<pcl::PointCloud<PointT>> plane_ins(new pcl::PointCloud<PointT>());
            for (auto &idx: cluster_res[i].indices) {
                plane_ins->push_back(cloud->points[idx]);
            }
            g3reg::ClusterFeature::Ptr feature(new g3reg::ClusterFeature(plane_ins));
            clusters.push_back(feature);
        }
    }
}

#endif //SRC_CLUSTERING_H
