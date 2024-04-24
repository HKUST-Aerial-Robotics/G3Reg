/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#pragma once

#include "../../../../../../../../usr/local/include/gtsam/geometry/Pose3.h"
#include "../../../../../../../../usr/local/include/gtsam/nonlinear/NonlinearFactor.h"
#include "front_end/gem/gemodel.h"

namespace gtsam {
    
    enum class RegularizationMethod { NONE, MIN_EIG, NORMALIZED_MIN_EIG, PLANE, LINE, FROBENIUS };

    Eigen::Matrix3d normalizeCov(const Eigen::Matrix3d &cov, RegularizationMethod regularization_method);

    class Point2PointFactor : public NoiseModelFactor1<Pose3> {

    private:
        typedef NoiseModelFactor1<Pose3> Base;

    public:
        Eigen::Vector3d msrc_, mtgt_; ///< src and tgt measurements

        Point2PointFactor() {} ///< Default constructor for serialization
        Point2PointFactor(Key X, Eigen::Vector3d src, Eigen::Vector3d tgt,
                      const SharedNoiseModel &model = nullptr) :
                Base(model, X) {
            msrc_ = src;
            mtgt_ = tgt;
        }

        ~Point2PointFactor() override {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Point2PointFactor(*this)));
        }

        Vector evaluateError(const Pose3 &X, boost::optional<Matrix &> H = boost::none) const;
    };

    class Point2PlaneFactor : public NoiseModelFactor1<Pose3> {

    private:
        typedef NoiseModelFactor1<Pose3> Base;

    public:
        Eigen::Vector3d msrc_, mtgt_, mnormal_; ///< src and tgt measurements

        Point2PlaneFactor() {} ///< Default constructor for serialization
        Point2PlaneFactor(Key X, Eigen::Vector3d src, Eigen::Vector3d tgt, Eigen::Vector3d normal,
                      const SharedNoiseModel &model = nullptr) :
                Base(model, X) {
            msrc_ = src;
            mtgt_ = tgt;
            mnormal_ = normal;
        }

        ~Point2PlaneFactor() override {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Point2PlaneFactor(*this)));
        }

        Vector evaluateError(const Pose3 &X, boost::optional<Matrix &> H = boost::none) const;
    };

    class Point2LineFactor : public NoiseModelFactor1<Pose3> {

    private:
        typedef NoiseModelFactor1<Pose3> Base;

    public:
        Eigen::Vector3d msrc_, mtgt_, mdirection_; ///< src and tgt measurements

        Point2LineFactor() {} ///< Default constructor for serialization
        Point2LineFactor(Key X, Eigen::Vector3d src, Eigen::Vector3d tgt, Eigen::Vector3d direction,
                          const SharedNoiseModel &model = nullptr) : Base(model, X) {
            msrc_ = src;
            mtgt_ = tgt;
            mdirection_ = direction;
        }

        ~Point2LineFactor() override {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Point2LineFactor(*this)));
        }

        Vector evaluateError(const Pose3 &X, boost::optional<Matrix &> H = boost::none) const;
    };

    class Gaussian2GaussianFactor : public NoiseModelFactor1<Pose3> {

    private:
        typedef NoiseModelFactor1<Pose3> Base;

    public:
        Eigen::Vector3d m_c1, m_c2;
        Eigen::Matrix3d m_cov1, m_cov2;
        
        Gaussian2GaussianFactor() {} ///< Default constructor for serialization
        Gaussian2GaussianFactor(Key X, Eigen::Vector3d c1, Eigen::Vector3d c2, Eigen::Matrix3d cov1, Eigen::Matrix3d cov2,
                                const SharedNoiseModel &model = nullptr, RegularizationMethod regularization_method = RegularizationMethod::NONE) :
                Base(model, X) {
            m_c1 = c1;
            m_c2 = c2;
            m_cov1 = normalizeCov(cov1, regularization_method);
            m_cov2 = normalizeCov(cov2, regularization_method);
        }

        ~Gaussian2GaussianFactor() override {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Gaussian2GaussianFactor(*this)));
        }

        Vector evaluateError(const Pose3 &X, boost::optional<Matrix &> H = boost::none) const;
    };


    class Point2GaussianFactor : public NoiseModelFactor1<Pose3> {

    private:
        typedef NoiseModelFactor1<Pose3> Base;

    public:
        Eigen::Vector3d m_c1, m_c2;
        Eigen::Matrix3d m_sqrtMahalanobis;

        Point2GaussianFactor() {} ///< Default constructor for serialization
        Point2GaussianFactor(Key X, Eigen::Vector3d c1, Eigen::Vector3d c2, Eigen::Matrix3d cov2,
                                const SharedNoiseModel &model = nullptr, RegularizationMethod regularization_method = RegularizationMethod::NONE) :
                Base(model, X) {
            m_c1 = c1;
            m_c2 = c2;
            Eigen::Matrix3d cov_normalized = normalizeCov(cov2, regularization_method);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_normalized);
            m_sqrtMahalanobis = es.eigenvalues().cwiseInverse().cwiseSqrt().asDiagonal() * es.eigenvectors().transpose();
        }

        ~Point2GaussianFactor() override {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Point2GaussianFactor(*this)));
        }

        Vector evaluateError(const Pose3 &X, boost::optional<Matrix &> H = boost::none) const;
    };

    Eigen::Matrix4d gncQuadricsSE3(const std::vector<g3reg::QuadricFeature::Ptr> &src, const std::vector<g3reg::QuadricFeature::Ptr> &tgt,
                                   clique_solver::Association &assoc, Eigen::Matrix4d init = Eigen::Matrix4d::Identity());

    Eigen::Matrix4d gncSE3(const Eigen::Matrix3Xd &src, const Eigen::Matrix3Xd &tgt);

    Eigen::Matrix4d svdSE3(const Eigen::Matrix3Xd &src, const Eigen::Matrix3Xd &tgt);

    Eigen::Matrix2d svdRot2d(const Eigen::Matrix<double, 2, Eigen::Dynamic>& X, const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y, const Eigen::Matrix<double, 1, Eigen::Dynamic>& W);
	
	Eigen::Matrix3d svdRot(const Eigen::Matrix<double, 3, Eigen::Dynamic> &X,
						   const Eigen::Matrix<double, 3, Eigen::Dynamic> &Y,
						   const Eigen::Matrix<double, 1, Eigen::Dynamic> &W);
}

