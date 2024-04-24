/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "utils/opt_utils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GncOptimizer.h>

using namespace gtsam;
using namespace g3reg;
using symbol_shorthand::X;  // pose

namespace gtsam{
	
	/**
	 * Helper function to use svd to estimate rotation.
	 * Method described here: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
	 * @param X
	 * @param Y
	 * @return a rotation matrix R
	 */
	Eigen::Matrix3d svdRot(const Eigen::Matrix<double, 3, Eigen::Dynamic> &X,
								  const Eigen::Matrix<double, 3, Eigen::Dynamic> &Y,
								  const Eigen::Matrix<double, 1, Eigen::Dynamic> &W) {
		// Assemble the correlation matrix H = X * Y'
		Eigen::Matrix3d H = X * W.asDiagonal() * Y.transpose();
		
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		
		if (U.determinant() * V.determinant() < 0) {
			V.col(2) *= -1;
		}
		
		return V * U.transpose();
	}

    Eigen::Matrix4d gncQuadricsSE3(const std::vector<g3reg::QuadricFeature::Ptr> &src,
                                   const std::vector<g3reg::QuadricFeature::Ptr> &tgt,
                                   clique_solver::Association &assoc, Eigen::Matrix4d init) {

        // Create a factor graph
        NonlinearFactorGraph graph;
        Values initial;
        initial.insert(X(0), Pose3(init));
        for (int i = 0; i < assoc.rows(); ++i) {
            QuadricFeature::Ptr src_quadric = src[assoc(i, 0)];
            QuadricFeature::Ptr tgt_quadric = tgt[assoc(i, 1)];
            Eigen::Vector3d tgt_center = tgt_quadric->center();
            Eigen::Vector3d src_center = src_quadric->center();
            noiseModel::Diagonal::shared_ptr noise = noiseModel::Unit::Create(3);
            if (src_quadric->type() == FeatureType::Plane && tgt_quadric->type() == FeatureType::Plane) {
                graph.add(Gaussian2GaussianFactor(X(0), src_center, tgt_center, src_quadric->sigma(),
                                                  tgt_quadric->sigma(), noise, RegularizationMethod::PLANE));
            } else if (src_quadric->type() == FeatureType::Line && tgt_quadric->type() == FeatureType::Line) {
                graph.add(Gaussian2GaussianFactor(X(0), src_center, tgt_center, src_quadric->sigma(),
                                                  tgt_quadric->sigma(), noise, RegularizationMethod::NONE));
            } else if (src_quadric->type() == FeatureType::Cluster && tgt_quadric->type() == FeatureType::Cluster){
                graph.add(Gaussian2GaussianFactor(X(0), src_center, tgt_center, src_quadric->sigma(),
                                                  tgt_quadric->sigma(), noise, RegularizationMethod::NONE));
            } else{
                std::cerr << "Unknown semantic type!" << std::endl;
            }
        }
		
		LevenbergMarquardtParams params;
//		params.setMaxIterations(10);
        GncParams<LevenbergMarquardtParams> gncParams(params);
//		gncParams.setMaxIterations(10);
        auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(graph,initial,gncParams);
        Values estimate = gnc.optimize();
        Eigen::Matrix4d T = estimate.at<Pose3>(X(0)).matrix();
        return T;
    }

    Eigen::Matrix4d gncSE3(const Eigen::Matrix3Xd &src, const Eigen::Matrix3Xd &tgt) {

        // Create a factor graph
        NonlinearFactorGraph graph;
        Values initial;
        initial.insert(X(0), Pose3(Eigen::Matrix4d::Identity()));
        noiseModel::Diagonal::shared_ptr noise = noiseModel::Unit::Create(3);
        assert(src.cols() == tgt.cols());
        for (int i = 0; i < src.cols(); ++i) {
            graph.add(Point2PointFactor(X(0), src.col(i), tgt.col(i), noise));
        }
        GncParams<LevenbergMarquardtParams> gncParams;
        auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(graph,initial,gncParams);
        Values estimate = gnc.optimize();
        Eigen::Matrix4d T = estimate.at<Pose3>(X(0)).matrix();
        return T;
    }

    Eigen::Matrix4d svdSE3(const Eigen::Matrix3Xd &src, const Eigen::Matrix3Xd &tgt) {
        Eigen::Vector3d src_mean = src.rowwise().mean();
        Eigen::Vector3d tgt_mean = tgt.rowwise().mean();
        const Eigen::Matrix3Xd& src_centered = src - src_mean.replicate(1, src.cols());
        const Eigen::Matrix3Xd& tgt_centered = tgt - tgt_mean.replicate(1, tgt.cols());
        Eigen::MatrixXd H = src_centered * tgt_centered.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd R_ = svd.matrixV() * svd.matrixU().transpose();
        if (R_.determinant() < 0) {
            Eigen::MatrixXd V = svd.matrixV();
            V.col(2) *= -1;
            R_ = V * svd.matrixU().transpose();
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block(0, 0, 3, 3) = R_;
        T.block(0, 3, 3, 1) = tgt_mean - R_ * src_mean;
        return T;
    }

    Eigen::Matrix2d svdRot2d(const Eigen::Matrix<double, 2, Eigen::Dynamic>& X,
                             const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                             const Eigen::Matrix<double, 1, Eigen::Dynamic>& W) {
        // Assemble the correlation matrix H = X * Y'
        Eigen::Matrix2d H = X * W.asDiagonal() * Y.transpose();

        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();

        if (U.determinant() * V.determinant() < 0) {
            V.col(1) *= -1;
        }

        return V * U.transpose();
    }

    Eigen::Matrix3d normalizeCov(const Eigen::Matrix3d &cov, RegularizationMethod regularization_method) {
        Eigen::Matrix3d cov_normalized;
        if (regularization_method == RegularizationMethod::NONE) {
            cov_normalized = cov;
        } else if (regularization_method == RegularizationMethod::FROBENIUS) {
            double lambda = 1e-3;
            Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d C_inv = C.inverse();
            cov_normalized.setZero();
            cov_normalized.template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
        } else {
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d values;

            switch (regularization_method) {
                default:
                    std::cerr << "here must not be reached" << std::endl;
                    abort();
                case RegularizationMethod::PLANE:
                    values = Eigen::Vector3d(1, 1, 1e-3);
                    break;
                case RegularizationMethod::LINE:
                    values = Eigen::Vector3d(1, 1e-3, 1e-3);
                    break;
                case RegularizationMethod::MIN_EIG:
                    values = svd.singularValues().array().max(1e-3);
                    break;
                case RegularizationMethod::NORMALIZED_MIN_EIG:
                    values = svd.singularValues() / svd.singularValues().maxCoeff();
                    values = values.array().max(1e-3);
                    break;
            }

            cov_normalized.setZero();
            cov_normalized.template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
        }
        return cov_normalized;
    }


    Vector Point2PointFactor::evaluateError(const Pose3 &X, boost::optional<Matrix &> H) const {
        const gtsam::Rot3 &R = X.rotation();
        gtsam::Vector3 mx = R * msrc_ + X.translation();
        gtsam::Vector3 error = mx - mtgt_;
        if (H) {
            *H = gtsam::Matrix(3, 6);
            (*H).block(0, 0, 3, 3) = -X.rotation().matrix() * skewSymmetric(msrc_);
            (*H).block(0, 3, 3, 3) = X.rotation().matrix();
        }
        return error;
    }

    Vector Point2PlaneFactor::evaluateError(const Pose3 &X, boost::optional<Matrix &> H) const {
        const gtsam::Rot3 &R = X.rotation();
        gtsam::Vector3 mx = R * msrc_ + X.translation();
        gtsam::Vector1 error((mx - mtgt_).dot(mnormal_));
        if (H) {
            *H = gtsam::Matrix(3, 6);
            (*H).block(0, 0, 3, 3) = -X.rotation().matrix() * skewSymmetric(msrc_);
            (*H).block(0, 3, 3, 3) = X.rotation().matrix();
            (*H) = mnormal_.transpose() * (*H);
        }
        return error;
    }

    Vector Point2LineFactor::evaluateError(const Pose3 &X, boost::optional<Matrix &> H) const {
        const gtsam::Rot3 &R = X.rotation();
        gtsam::Vector3 mx = R * msrc_ + X.translation();
        gtsam::Vector3 error = (Eigen::Matrix3d::Identity() - mdirection_ * mdirection_.transpose()) * (mx - mtgt_);
        if (H) {
            *H = gtsam::Matrix(3, 6);
            (*H).block(0, 0, 3, 3) = -X.rotation().matrix() * skewSymmetric(msrc_);
            (*H).block(0, 3, 3, 3) = X.rotation().matrix();
            (*H) = (Eigen::Matrix3d::Identity() - mdirection_ * mdirection_.transpose()) * (*H);
        }
        return error;
    }

    Vector Gaussian2GaussianFactor::evaluateError(const Pose3 &X, boost::optional<Matrix &> H) const {
        const gtsam::Rot3 &R = X.rotation();
        Eigen::Matrix3d cov = R.matrix() * m_cov1 * R.matrix().transpose() + m_cov2;
        Eigen::Vector3d mx = R * m_c1 + X.translation();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        const Eigen::Matrix3d sqrtMahalanobis = es.eigenvalues().cwiseInverse().cwiseSqrt().asDiagonal() * es.eigenvectors().transpose();
        gtsam::Vector3 error = sqrtMahalanobis * (mx - m_c2);
        if (H) {
            *H = gtsam::Matrix(3, 6);
            (*H).block(0, 0, 3, 3) = -X.rotation().matrix() * skewSymmetric(m_c1);
            (*H).block(0, 3, 3, 3) = X.rotation().matrix();
            (*H) = sqrtMahalanobis * (*H);
        }
        return error;
    }

    Vector Point2GaussianFactor::evaluateError(const Pose3 &X, boost::optional<Matrix &> H) const {
        const gtsam::Rot3 &R = X.rotation();
        Eigen::Vector3d mx = R * m_c1 + X.translation();
        gtsam::Vector3 error = m_sqrtMahalanobis * (mx - m_c2);
        if (H) {
            *H = gtsam::Matrix(3, 6);
            (*H).block(0, 0, 3, 3) = -X.rotation().matrix() * skewSymmetric(m_c1);
            (*H).block(0, 3, 3, 3) = X.rotation().matrix();
            (*H) = m_sqrtMahalanobis * (*H);
        }
        return error;
    }
}