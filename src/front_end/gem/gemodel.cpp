/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "front_end/gem/gemodel.h"
#include "front_end/gem/bounding_box.h"
#include "robot_utils/random_utils.h"

namespace g3reg {


    double QuadricFeature::score() const {
        double score = 0.0;
        Eigen::Vector3d lambda = eigen_values();
        lambda = lambda / lambda.sum();
        if (type_ == FeatureType::Cluster){
//            use Omnivariance
            score = lambda[0] * lambda[1] * lambda[2];
//            score = lambda_[0] * lambda_[1] * lambda_[2];
        } else if (type_ == FeatureType::Plane){
            score = lambda_[1] * lambda_[2];
        } else if (type_ == FeatureType::Line){
            score = lambda_[2];
        }
        return score;
    }

    void QuadricFeature::fitting(){
        if (!cloud_)
            return;
        std::string method = "OBB";
        if (method=="PCA"){
            // compute the oriented bounding box
            Rwb = umat_;
            Rwb.col(2) = Rwb.col(0).cross(Rwb.col(1));
            Eigen::Vector3d bottomLeft_w = Rwb.transpose() * Eigen::Vector3d(cloud_->points[0].x, cloud_->points[0].y, cloud_->points[0].z);
            Eigen::Vector3d topRight_w = Rwb.transpose() * Eigen::Vector3d(cloud_->points[0].x, cloud_->points[0].y, cloud_->points[0].z);
            for (const auto &point : cloud_->points){
                Eigen::Vector3d point_w = Eigen::Vector3d(point.x, point.y, point.z);
                Eigen::Vector3d point_local = Rwb.transpose() * point_w;
                bottomLeft_w = bottomLeft_w.cwiseMin(point_local);
                topRight_w = topRight_w.cwiseMax(point_local);
            }
            center_geo_ = Rwb * (bottomLeft_w + topRight_w) / 2;
            size_ = (topRight_w - bottomLeft_w).cwiseAbs();
        } else{
            BBoxUtils::RotatedRect rect = BBoxUtils::fittingBoundary(*cloud_, normal_);
            Rwb = rect.basis_;
            size_ = rect.size_;
            center_geo_ =  Rwb * (rect.bottomLeft + rect.topRight) / 2;
        }
    }


    Eigen::VectorXd QuadricFeature::eigenvalue_feature() const{
        Eigen::Vector3d ev = lambda_ / lambda_.sum();
        // sort ev in descending order
        std::vector<double> ev_vec = {ev[0], ev[1], ev[2]};
        std::sort(ev_vec.begin(), ev_vec.end(), std::greater<double>());

        const double& e1 = ev_vec[0], e2 = ev_vec[1], e3 = ev_vec[2];
        Eigen::VectorXd ev_feature = Eigen::VectorXd::Zero(8);
        ev_feature << (e1-e2)/e1, (e2-e3)/e1, e3/e1, pow(e1 * e2 * e3, 1.0/3.0), (e1-e3)/e1, -e1 * log(e1) - e2 * log(e2) - e3 * log(e3), e1 + e2 + e3, e3/(e1+e2+e3);
        return ev_feature;
    }

    Eigen::Matrix3d QuadricFeature::normalized_cov(Eigen::Matrix3d& cov, Eigen::Vector3d& ev, double chi2) const {
        ev = (size_/2).cwiseAbs2() / chi2;
        cov = Rwb * ev.asDiagonal() * Rwb.transpose();
        return cov;
    }

    void QuadricFeature::transform(const Eigen::Matrix4d &T) {
        center_geo_ = T.block<3, 3>(0, 0) * center_geo_ + T.block<3, 1>(0, 3);
        center_ = T.block<3, 3>(0, 0) * center_ + T.block<3, 1>(0, 3);
        sigma_ = T.block<3, 3>(0, 0) * sigma_ * T.block<3, 3>(0, 0).transpose();
        Rwb = T.block<3, 3>(0, 0) * Rwb;
        umat_ = T.block<3, 3>(0, 0) * umat_;
        normal_ = T.block<3, 3>(0, 0) * normal_;
        direction_ = T.block<3, 3>(0, 0) * direction_;
        pcl::transformPointCloud(*cloud_, *cloud_, T);
    }

    clique_solver::GraphVertex::Ptr QuadricFeature::vertex(){
    
		Eigen::Vector3d center = config::use_bbox_center ? center_geo_ : center_;
		
        Eigen::Matrix3d cov;
        Eigen::Vector3d ev;
        if (config::use_pseudo_cov){
            normalized_cov(cov, ev, config::volume_chi2);
        } else{
            cov = sigma_;
            ev = lambda_;
        }

        clique_solver::GraphVertex::Ptr vertex = clique_solver::create_vertex(center, config::vertex_info, cov, ev.maxCoeff());
        return vertex;
    }

    bool LineFeature::Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        cloud_ = cloud;

        solveCovMat(*cloud_, center_, sigma_);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
        lambda_ = saes.eigenvalues();
        if (lambda_(2) / lambda_(0) < config::eigenvalue_thresh ) {
            return false;
        }

        umat_ = saes.eigenvectors();
        normal_ = umat_.col(2); // direction of the line, the maximum eigenvalue
        direction_ = umat_.col(2);
        point_a_ = center_ + normal_ * std::sqrt(lambda_(2)) * 2;
        point_b_ = center_ - normal_ * std::sqrt(lambda_(2)) * 2;
        return true;
    }

    LineFeature::Ptr LineFeature::Random(){
        Eigen::Vector3d p1 = robot_utils::randomTranslation(1.0), p2 = robot_utils::randomTranslation(1.0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < 20; ++i){
            pcl::PointXYZ point;
            point.x = p1(0) + (p2(0) - p1(0)) * i / 20.0;
            point.y = p1(1) + (p2(1) - p1(1)) * i / 20.0;
            point.z = p1(2) + (p2(2) - p1(2)) * i / 20.0;
            cloud->push_back(point);
        }
        LineFeature::Ptr line(new LineFeature(cloud));
        return line;
    }

    bool SurfaceFeature::merge(const Voxel& voxel){
        int N1 = cloud_->size();
        int N2 = voxel.cloud()->size();
        Eigen::Vector3d center1 = center_;
        Eigen::Vector3d center2 = voxel.center();
        Eigen::Matrix3d sigma1 = sigma_;
        Eigen::Matrix3d sigma2 = voxel.sigma();

        *cloud_ += *voxel.cloud();
        mergeGaussian(center1, sigma1, N1, center2, sigma2, N2, center_, sigma_);

        //solveCovMat(*cloud_, center_, sigma_);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
        lambda_ = saes.eigenvalues();
        umat_ = saes.eigenvectors();
        normal_ = saes.eigenvectors().col(0);

        voxels_.push_back(voxel.loc());
        return true;
    }

    bool SurfaceFeature::consistent(const Voxel& voxel){
        if (voxel.type() != FeatureType::Plane){
            return false;
        }

        //point to plane distance
        if (abs((voxel.center() - center_).dot(normal_)) > config::plane_distance_thresh){
            return false;
        }
        // normal
        if (abs(voxel.normal().dot(normal_)) < config::plane_normal_thresh){
            return false;
        }

        return true;
    }

    bool SurfaceFeature::merge(const SurfaceFeature &surface) {

        int N1 = cloud_->size();
        int N2 = surface.cloud()->size();
        Eigen::Vector3d center1 = center_;
        Eigen::Vector3d center2 = surface.center();
        Eigen::Matrix3d sigma1 = sigma_;
        Eigen::Matrix3d sigma2 = surface.sigma();

        *cloud_ += *surface.cloud();
        center_ = (N1 * center1 + N2 * center2) / (N1 + N2);
        sigma_ = ((sigma1 + center1 * center1.transpose()) * N1 + (sigma2 + center2 * center2.transpose()) * N2) / (N1 + N2) - center_ * center_.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sigma_);
        lambda_ = saes.eigenvalues();
        umat_ = saes.eigenvectors();
        normal_ = saes.eigenvectors().col(0);
        for (auto& loc : surface.voxels()){
            voxels_.push_back(loc);
        }
        return true;
    }

    bool SurfaceFeature::consistent(const SurfaceFeature& surface){
        //point to plane distance
        if (abs((surface.center() - center_).dot(normal_)) > config::plane_distance_thresh){
            return false;
        }
        // normal
        if (abs(surface.normal().dot(normal_)) < config::plane_normal_thresh){
            return false;
        }

        return true;
    }

    SurfaceFeature::Ptr SurfaceFeature::Random(){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector3d p1 = robot_utils::randomTranslation(1.0), p2 = robot_utils::randomTranslation(1.0), p3 = robot_utils::randomTranslation(1.0);
        for (int i = 0; i < 100; ++i){
            Eigen::Vector2d weight = robot_utils::randomTranslation(1.0).head(2);
            Eigen::Vector3d point = weight(0) * p1 + weight(1) * p2 + (1 - weight(0) - weight(1)) * p3;
            pcl::PointXYZ p;
            p.x = point(0); p.y = point(1); p.z = point(2);
            cloud->push_back(p);
        }
        SurfaceFeature::Ptr plane(new SurfaceFeature(cloud));
        return plane;
    }

    ClusterFeature::Ptr ClusterFeature::Random(){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector3d scale = robot_utils::randomTranslation(1.0);
        for (int i = 0; i < 20; ++i){
            Eigen::Vector3d point = robot_utils::randomTranslation(1.0).cwiseProduct(scale);
            pcl::PointXYZ p;
            p.x = point(0); p.y = point(1); p.z = point(2);
            cloud->push_back(p);
        }
        ClusterFeature::Ptr cluster(new ClusterFeature(cloud));
        return cluster;
    }
}