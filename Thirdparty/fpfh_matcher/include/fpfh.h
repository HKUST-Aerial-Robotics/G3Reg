/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#pragma once

#include <boost/smart_ptr/shared_ptr.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>

namespace teaser {

    using FPFHCloud = pcl::PointCloud<pcl::FPFHSignature33>;
    using FPFHCloudPtr = pcl::PointCloud<pcl::FPFHSignature33>::Ptr;

    class FPFHEstimation {
    public:
        FPFHEstimation()
                : fpfh_estimation_(
                new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>) {};

        /**
         * Compute FPFH features.
         *
         * @return A shared pointer to the FPFH feature point cloud
         * @param input_cloud
         * @param normal_search_radius Radius for estimating normals
         * @param fpfh_search_radius Radius for calculating FPFH (needs to be at least normalSearchRadius)
         */
        FPFHCloudPtr computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                         double normal_search_radius = 1.0,
                                         double fpfh_search_radius = 2.5) {

            // Intermediate variables
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            teaser::FPFHCloudPtr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

            // Estimate normals
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
            normalEstimation.setInputCloud(input_cloud);
            normalEstimation.setRadiusSearch(normal_search_radius);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
            normalEstimation.setSearchMethod(kdtree);
            normalEstimation.compute(*normals);

            // Estimate FPFH
            setInputCloud(input_cloud);
            setInputNormals(normals);
            setSearchMethod(kdtree);
            setRadiusSearch(fpfh_search_radius);
            compute(*descriptors);

            return descriptors;
        }

        /**
         * Return the pointer to the underlying pcl::FPFHEstimation object
         * @return
         */
        inline pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr
        getImplPointer() const {
            return fpfh_estimation_;
        }

    private:
        // pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation_;
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation_;

        /**
         * Wrapper function for the corresponding PCL function.
         * @param output_cloud
         */
        void compute(pcl::PointCloud<pcl::FPFHSignature33> &output_cloud) {
            fpfh_estimation_->compute(output_cloud);
        }

        /**
         * Wrapper function for the corresponding PCL function.
         * @param input_cloud
         */
        void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
            fpfh_estimation_->setInputCloud(input_cloud);
        }

        /**
         * Wrapper function for the corresponding PCL function.
         * @param input_normals
         */
        void setInputNormals(pcl::PointCloud<pcl::Normal>::Ptr input_normals) {
            fpfh_estimation_->setInputNormals(input_normals);
        }

        /**
         * Wrapper function for the corresponding PCL function.
         * @param search_method
         */
        void setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method) {
            fpfh_estimation_->setSearchMethod(search_method);
        }

        /**
         * Wrapper function for the corresponding PCL function.
         */
        void setRadiusSearch(double r) { fpfh_estimation_->setRadiusSearch(r); }
    };

} // namespace teaser
