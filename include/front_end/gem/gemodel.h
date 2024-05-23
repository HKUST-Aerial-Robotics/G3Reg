/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SEMANTIC_FEATURE_H
#define SEMANTIC_FEATURE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "utils/config.h"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include "robot_utils/random_utils.h"
#include "front_end/gem/voxel.h"

namespace g3reg {

    class QuadricFeature {
    public:
        typedef std::shared_ptr<QuadricFeature> Ptr;

        QuadricFeature() {
            cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        }

        QuadricFeature(Eigen::Vector3d center, Eigen::Matrix3d sigma = Eigen::Matrix3d::Identity()) {
            cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            center_ = center;
            sigma_ = sigma;
        }

        QuadricFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
            cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*cloud, *cloud_);
            solveCovMat(*cloud_, center_, sigma_);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
            lambda_ = saes.eigenvalues();
            umat_ = saes.eigenvectors();
            normal_ = umat_.col(0);
            direction_ = umat_.col(2);
        }

        Eigen::Vector3d center() const { return center_; }

        Eigen::Vector3d normal() const { return normal_; }

        Eigen::Matrix3d sigma() const { return sigma_; }

        Eigen::Vector3d eigen_values() const { return lambda_; }

        Eigen::Vector3d direction() const { return direction_; }

        FeatureType type() const { return type_; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const { return cloud_; }

        Eigen::Vector3d center_geo() const { return center_geo_; }

        Eigen::Vector3d size() const { return size_; }

        double volume() const { return size_.x() * size_.y() * size_.z(); }

        Eigen::Matrix3d rotation() const { return Rwb; }

        void set_center(const Eigen::Vector3d &center) { center_ = center; }

        void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) { cloud_ = cloud; }

        void set_center_geo(const Eigen::Vector3d &center_geo) { center_geo_ = center_geo; }

        void set_size(const Eigen::Vector3d &size) { size_ = size; }

        void set_rotation(const Eigen::Matrix3d &rotation) { Rwb = rotation; }

        void fitting();

        double score() const;

        Eigen::VectorXd eigenvalue_feature() const;

        Eigen::Matrix3d normalized_cov(Eigen::Matrix3d &cov, Eigen::Vector3d &ev, double chi2) const;

        void transform(const Eigen::Matrix4d &T);

        clique_solver::GraphVertex::Ptr vertex();

    protected:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        Eigen::Matrix3d sigma_; // covariance matrix
        Eigen::Matrix3d umat_, Rwb; // umat_ is the eigen vectors, Rwb is the rotation matrix of bbox
        Eigen::Vector3d lambda_; // eigenvalues, sorted in ascending order
        Eigen::Vector3d center_, normal_, center_geo_, direction_;
        Eigen::Vector3d size_; // half of the length of the three edges, x, y, z
        FeatureType type_;
    };

    // pole like feature
    class LineFeature : public QuadricFeature {
    public:
        typedef std::shared_ptr<LineFeature> Ptr;

        LineFeature() : QuadricFeature() {
            type_ = FeatureType::Line;
        }

        LineFeature(const Eigen::Vector3d &pa, const Eigen::Vector3d &pb,
                    const Eigen::Matrix3d &sigma) : point_a_(pa), point_b_(pb), QuadricFeature() {
            type_ = FeatureType::Line;
            sigma_ = sigma;
        }

        LineFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) : QuadricFeature(cloud) {
            type_ = FeatureType::Line;
        }

        bool Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

        Eigen::Vector3d point_a() const { return point_a_; }

        Eigen::Vector3d point_b() const { return point_b_; }

        static LineFeature::Ptr Random();

    private:
        Eigen::Vector3d point_a_, point_b_;
    };

    class ClusterFeature : public QuadricFeature {

    public:
        typedef std::shared_ptr<ClusterFeature> Ptr;

        ClusterFeature() : QuadricFeature() {
            type_ = FeatureType::Cluster;
        }

        ClusterFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) : QuadricFeature(cloud) {
            type_ = FeatureType::Cluster;
            normal_ = Eigen::Vector3d::UnitZ();
        }

        static ClusterFeature::Ptr Random();
    };

    // road, sidewalk, wall, etc.
    class SurfaceFeature : public QuadricFeature {
    public:
        typedef std::shared_ptr<SurfaceFeature> Ptr;

        SurfaceFeature() : QuadricFeature() {
            type_ = FeatureType::Plane;
            voxels_.clear();
        }

        SurfaceFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) : QuadricFeature(cloud) {
            type_ = FeatureType::Plane;
        }

        SurfaceFeature(const Eigen::Vector3d &center, const Eigen::Vector3d &normal,
                       const Eigen::Matrix3d &sigma) : SurfaceFeature() {

            center_ = center;
            normal_ = normal;
            sigma_ = sigma;
            cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
            lambda_ = saes.eigenvalues();
            umat_ = saes.eigenvectors();
        }

        bool merge(const Voxel &voxel);

        bool consistent(const Voxel &voxel);

        bool merge(const SurfaceFeature &surface);

        bool consistent(const SurfaceFeature &surface);

        static SurfaceFeature::Ptr Random();

        std::vector<VoxelKey> voxels() const { return voxels_; }

        void push_back(const VoxelKey &loc) { voxels_.push_back(loc); }

    private:
        std::vector<VoxelKey> voxels_;
    };

} // namespace g3reg


#endif