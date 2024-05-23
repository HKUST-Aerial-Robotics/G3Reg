/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_QUATRO_H
#define SRC_QUATRO_H

#include <Eigen/Core>
#include "registration.h"

namespace quatro {

    class QuatroSolver {
    public:
        QuatroSolver(teaser::RobustRegistrationSolver::Params params) : params_(params) {}

        Eigen::Matrix3d solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic> &v1,
                                         const Eigen::Matrix<double, 3, Eigen::Dynamic> &v2);

        void solveForRotation2D(const Eigen::Matrix<double, 2, Eigen::Dynamic> &src,
                                const Eigen::Matrix<double, 2, Eigen::Dynamic> &dst, Eigen::Matrix2d *rotation,
                                Eigen::Matrix<bool, 1, Eigen::Dynamic> *inliers);

        // For leveraging IMU data
        void setPreEstimatedRyRx(Eigen::Matrix4d &estimated_RyRx) {
            estimated_RyRx_ = estimated_RyRx.block<3, 3>(0, 0);
            using_pre_estimated_RyRx_ = true;
        }

        Eigen::Matrix<bool, 1, Eigen::Dynamic> getRotationInliersMask() {
            return rotation_inliers_mask_;
        }

        Eigen::Matrix3d getRotationSolution() {
            return rotation_;
        }

    private:
        //Params.
        teaser::RobustRegistrationSolver::Params params_;
        Eigen::Matrix3d rotation_;
        Eigen::Matrix<bool, 1, Eigen::Dynamic> rotation_inliers_mask_;

        bool using_pre_estimated_RyRx_ = false;
        Eigen::Matrix3d estimated_RyRx_ = Eigen::Matrix3d::Identity();
        double cost_;
    };

}


#endif //SRC_QUATRO_H
