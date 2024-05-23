/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#ifndef VOXEL_H
#define VOXEL_H

#include <vector>
#include <cstdlib>
#include <algorithm>
#include <stdio.h>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "utils/config.h"
#include "robot_utils/eigen_types.h"

using VoxelKey = std::tuple<int, int, int>;

enum class FeatureType {
    None = -1,
    Cluster = 0,
    Line = 1,
    Plane = 2,
    Free = 3,
    Unknown = 4
};

inline std::ostream &operator<<(std::ostream &os, const FeatureType &type) {
    switch (type) {
        case FeatureType::None:
            os << "None";
            break;
        case FeatureType::Cluster:
            os << "Cluster";
            break;
        case FeatureType::Line:
            os << "Line";
            break;
        case FeatureType::Plane:
            os << "Plane";
            break;
        case FeatureType::Free:
            os << "Free";
            break;
        case FeatureType::Unknown:
            os << "Unknown";
            break;
        default:
            os << "Invalid";
            break;
    }
    return os;
}

template<typename PointT, typename T>
void solveCovMat(const pcl::PointCloud<PointT> &cloud, Eigen::Matrix<T, 3, 1> &mu,
                 Eigen::Matrix<T, 3, 3> &cov) {
    mu.setZero();
    cov.setZero();
    Eigen::Matrix<T, 3, 1> point;
    auto N = cloud.size();
    for (int i = 0; i < N; ++i) {
        point = cloud.points[i].getVector3fMap().template cast<T>();
        mu += point;
        cov += point * point.transpose();
    }
    mu /= N;
    cov.noalias() = cov / N - mu * mu.transpose();
}

namespace g3reg_utils {
    template<typename PointT, typename T>
    void solveCenter(const pcl::PointCloud<PointT> &cloud, Eigen::Matrix<T, 3, 1> &mu) {
        mu.setZero();
        Eigen::Matrix<T, 3, 1> point;
        auto N = cloud.size();
        for (int i = 0; i < N; ++i) {
            point = cloud.points[i].getVector3fMap().template cast<T>();
            mu += point;
        }
        mu /= N;
    }
}

template<typename T>
void mergeGaussian(const Eigen::Matrix<T, 3, 1> &center1, const Eigen::Matrix<T, 3, 3> &sigma1, int N1,
                   const Eigen::Matrix<T, 3, 1> &center2, const Eigen::Matrix<T, 3, 3> &sigma2, int N2,
                   Eigen::Matrix<T, 3, 1> &center_, Eigen::Matrix<T, 3, 3> &sigma_) {
    center_ = (N1 * center1 + N2 * center2) / (N1 + N2);
    sigma_ = ((sigma1 + center1 * center1.transpose()) * N1 + (sigma2 + center2 * center2.transpose()) * N2) /
             (N1 + N2) - center_ * center_.transpose();
}

template<typename PointT>
VoxelKey point_to_voxel_key(const PointT &point, const Eigen::Vector3f &voxel_size) {
    int x = static_cast<int>(std::floor(point.x / voxel_size.x()));
    int y = static_cast<int>(std::floor(point.y / voxel_size.y()));
    int z = static_cast<int>(std::floor(point.z / voxel_size.z()));
    return std::make_tuple(x, y, z);
}

template<typename T>
VoxelKey point_to_voxel_key(const Eigen::Matrix<T, 3, 1> &point, const Eigen::Vector3f &voxel_size) {
    int x = static_cast<int>(std::floor(point.x() / voxel_size.x()));
    int y = static_cast<int>(std::floor(point.y() / voxel_size.y()));
    int z = static_cast<int>(std::floor(point.z() / voxel_size.z()));
    return std::make_tuple(x, y, z);
}

class Voxel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Voxel> Ptr;

    Voxel(const VoxelKey key, const FeatureType type) : type_(type) {
        key_ = key;
        center_.setZero();
        normal_.setZero();
        sigma_.setZero();
        instance_id = -1;
    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const {
        return cloud_.makeShared();
    }

    static void getNeighbors(const VoxelKey &loc, std::vector<VoxelKey> &neighbors) {
        neighbors.clear();
        neighbors.reserve(27); // 3^3
        int64_t x = std::get<0>(loc), y = std::get<1>(loc), z = std::get<2>(loc);
        for (int64_t i = x - 1; i <= x + 1; ++i) {
            for (int64_t j = y - 1; j <= y + 1; ++j) {
                for (int64_t k = z - 1; k <= z + 1; ++k) {
                    neighbors.emplace_back(i, j, k);
                }
            }
        }
    }

    void getNeighbors(std::vector<VoxelKey> &neighbors) {
        getNeighbors(key_, neighbors);
    }

    const Eigen::Vector3d &center() const {
        return center_;
    }

    const Eigen::Vector3d &normal() const {
        return normal_;
    }

    const Eigen::Vector3d &direction() const {
        return direction_;
    }

    const Eigen::Matrix3d &sigma() const {
        return sigma_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() {
        return cloud_.makeShared();
    }

    void insertPoint(const pcl::PointXYZ &point) {
        cloud_.push_back(point);
    }

    bool parse() {
        if (cloud_.size() < 10) {
            g3reg_utils::solveCenter(cloud_, center_);
            return false;
        }

        solveCovMat(cloud_, center_, sigma_);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
        lambda_ = saes.eigenvalues();
        normal_ = saes.eigenvectors().col(0);
        if (lambda_(1) / lambda_(0) < g3reg::config.eigenvalue_thresh) {
            return false;
        }
        return true;
    }

    void solveCenter() {
        g3reg_utils::solveCenter(cloud_, center_);
    }

    FeatureType type() const {
        return type_;
    }

    void setSemanticType(const FeatureType type) {
        type_ = type;
    }

    VoxelKey loc() const {
        return key_;
    }

    void setDirection(const Eigen::Vector3d &direction) {
        direction_ = direction;
    }

    void setNormal(const Eigen::Vector3d &normal) {
        normal_ = normal;
    }

public:
    int instance_id;

private:
    VoxelKey key_;
    Eigen::Vector3f voxel_size_;
    Eigen::Vector3d center_, normal_, lambda_, direction_;
    Eigen::Matrix3d sigma_, eigen_vectors_; // eigen_vectors_ is in the ascending order of eigenvalues
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    FeatureType type_;
};

using VoxelMap = std::unordered_map<VoxelKey, Voxel::Ptr, robot_utils::Vec3dHash>;

template<typename PointT>
void cutCloud(const pcl::PointCloud<PointT> &cloud, const FeatureType type,
              const Eigen::Vector3f &voxel_size, VoxelMap &voxel_map) {
    for (size_t i = 0; i < cloud.size(); i++) {
        VoxelKey position = point_to_voxel_key(cloud.points[i], voxel_size);
        VoxelMap::iterator voxel_iter = voxel_map.find(position);
        if (voxel_iter != voxel_map.end()) {
            voxel_iter->second->insertPoint(cloud.points[i]);
        } else {
            Voxel::Ptr voxel = Voxel::Ptr(new Voxel(position, type));
            voxel->insertPoint(cloud.points[i]);
            voxel_map.insert(std::make_pair(position, voxel));
        }
    }
}


#endif