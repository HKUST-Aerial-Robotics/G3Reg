#include "back_end/teaser/quatro.h"
#include "utils/opt_utils.h"

namespace quatro {


    Eigen::Matrix3d QuatroSolver::solveForRotation(
            const Eigen::Matrix<double, 3, Eigen::Dynamic> &v1,
            const Eigen::Matrix<double, 3, Eigen::Dynamic> &v2) {

        rotation_inliers_mask_.resize(1, v1.cols());

        Eigen::Matrix2d rotation_2d;
        Eigen::Matrix<double, 2, Eigen::Dynamic> src_2d;
        Eigen::Matrix<double, 2, Eigen::Dynamic> dst_2d;
        // XY Coordinates for calculate yaw
        src_2d.resize(2, v1.cols());
        dst_2d.resize(2, v2.cols());
        src_2d = v1.topRows(2);
        dst_2d = v2.topRows(2);

        Eigen::Matrix3d rot_yaw = Eigen::Matrix3d::Identity();
        rotation_2d = Eigen::Matrix2d::Identity();
        solveForRotation2D(src_2d, dst_2d, &rotation_2d, &rotation_inliers_mask_);
        rot_yaw.block<2, 2>(0, 0) = rotation_2d;
        rotation_ = rot_yaw;

        /***
        * Once the prior pose is given by INS (by the function `setPreEstimatedRyRx()` ),
        * RyRx is multiplied by estimated Rz, i.e. Rz * RyRx
        * Note that one calls the function setPreEstimatedRyRx(),
        * then `using_pre_estimated_RyRx_` automatically turns into true
        */
        if (using_pre_estimated_RyRx_) {
            rotation_ = rotation_ * estimated_RyRx_;
        }
        return rotation_;
    }

    void QuatroSolver::solveForRotation2D(const Eigen::Matrix<double, 2, Eigen::Dynamic> &src,
                                          const Eigen::Matrix<double, 2, Eigen::Dynamic> &dst,
                                          Eigen::Matrix2d *rotation,
                                          Eigen::Matrix<bool, 1, Eigen::Dynamic> *inliers) {
        assert(rotation);                 // make sure R is not a nullptr
        assert(src.cols() == dst.cols()); // check dimensions of input data
        assert(params_.rotation_gnc_factor >
               1);   // make sure mu will increase        gnc_factor -> rotation_gnc_factor
        assert(params_.noise_bound != 0); // make sure noise sigma is not zero
        if (inliers) {
            assert(inliers->cols() == src.cols());
        }

        // Prepare some variables
        size_t match_size = src.cols(); // number of correspondences

        double mu = 1; // arbitrary starting mu

        double prev_cost = std::numeric_limits<double>::infinity();
        cost_ = std::numeric_limits<double>::infinity();
        static double noise_bound_sq = std::pow(params_.noise_bound, 2);
        if (noise_bound_sq < 1e-16) {
            noise_bound_sq = 1e-2;
        }
        TEASER_DEBUG_INFO_MSG("GNC rotation estimation noise bound:" << params_.noise_bound);
        TEASER_DEBUG_INFO_MSG("GNC rotation estimation noise bound squared:" << noise_bound_sq);

        Eigen::Matrix<double, 2, Eigen::Dynamic> diffs(2, match_size);
        Eigen::Matrix<double, 1, Eigen::Dynamic> weights(1, match_size);
        weights.setOnes(1, match_size);
        Eigen::Matrix<double, 1, Eigen::Dynamic> residuals_sq(1, match_size);

        // Loop for performing GNC-TLS
        for (size_t i = 0; i < params_.rotation_max_iterations; ++i) {   //max_iterations  = >rotation_max_iterations

            // Fix weights and perform SVD rotation estimation
            *rotation = gtsam::svdRot2d(src, dst, weights);

            // Calculate residuals squared
            diffs = (dst - (*rotation) * src).array().square();
            residuals_sq = diffs.colwise().sum();
            if (i == 0) {
                // Initialize rule for mu
                double max_residual = residuals_sq.maxCoeff();
                mu = 1 / (2 * max_residual / noise_bound_sq - 1);
                // Degenerate case: mu = -1 because max_residual is very small
                // i.e., little to none noise
                if (mu <= 0) {
                    TEASER_DEBUG_INFO_MSG(
                            "GNC-TLS terminated because maximum residual at initialization is very small.");
                    break;
                }
            }
            // Fix R and solve for weights in closed form
            double th1 = (mu + 1) / mu * noise_bound_sq;
            double th2 = mu / (mu + 1) * noise_bound_sq;
            cost_ = 0;
            for (size_t j = 0; j < match_size; ++j) {
                // Also calculate cost in this loop
                // Note: the cost calculated is using the previously solved weights
                cost_ += weights(j) * residuals_sq(j);

                if (residuals_sq(j) >= th1) {
                    weights(j) = 0;
                } else if (residuals_sq(j) <= th2) {
                    weights(j) = 1;
                } else {
                    weights(j) = sqrt(noise_bound_sq * mu * (mu + 1) / residuals_sq(j)) - mu;
                    assert(weights(j) >= 0 && weights(j) <= 1);
                }
            }
            // Calculate cost
            double cost_diff = std::abs(cost_ - prev_cost);

            // Increase mu
            mu = mu * params_.rotation_gnc_factor;   //gnc_factor -> rotation_gnc_factor
            prev_cost = cost_;

            if (cost_diff < params_.rotation_cost_threshold) {
                TEASER_DEBUG_INFO_MSG("GNC-TLS solver terminated due to cost convergence.");
                TEASER_DEBUG_INFO_MSG("Cost diff: " << cost_diff);
                TEASER_DEBUG_INFO_MSG("Iterations: " << i);
                break;
            }
        }

        if (inliers) {
            for (size_t i = 0; i < weights.cols(); ++i) {
                (*inliers)(0, i) = weights(0, i) >= 0.4;
            }
        }
    }
}