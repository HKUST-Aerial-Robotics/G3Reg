/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "back_end/ransac/ransac.h"
#include <vector>
#include <random>
#include <Eigen/Geometry>
#include "utils/opt_utils.h"

using namespace std;
using namespace clique_solver;

namespace ransac {

    Eigen::Matrix4d
    ransac_registration(const std::vector<Eigen::Vector3d> &src_points, const std::vector<Eigen::Vector3d> &tgt_points,
                        const Eigen::MatrixX2i &associations, RansacParams params) {
        int num_points = associations.rows();
        int best_inliers = -1;
        Eigen::Matrix4d best_transform = Eigen::Matrix4d::Identity();
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, num_points - 1);

        int inliers_to_end = params.inliers_to_end * num_points;
#pragma omp parallel for
        for (int iter = 0; iter < params.max_iterations; ++iter) {
            if (best_inliers >= inliers_to_end) continue; // End early if enough inliers have been found

            int i = dist(rng), j = dist(rng), l = dist(rng), src_i_idx, src_j_idx, src_k_idx, tgt_i_idx, tgt_j_idx, tgt_k_idx;
            src_i_idx = associations(i, 0), src_j_idx = associations(j, 0), src_k_idx = associations(l, 0);
            tgt_i_idx = associations(i, 1), tgt_j_idx = associations(j, 1), tgt_k_idx = associations(l, 1);
            // Check if there are overlaps in the indices
            if (src_i_idx == src_j_idx || src_i_idx == src_k_idx || src_j_idx == src_k_idx ||
                tgt_i_idx == tgt_j_idx || tgt_i_idx == tgt_k_idx || tgt_j_idx == tgt_k_idx)
            {
                continue;
            }

            const auto& src_i = src_points[associations(i, 0)];
            const auto& tgt_i = tgt_points[associations(i, 1)];

            const auto& src_j = src_points[associations(j, 0)];
            const auto& tgt_j = tgt_points[associations(j, 1)];

            const auto& src_k = src_points[associations(l, 0)];
            const auto& tgt_k = tgt_points[associations(l, 1)];

            float src_ij_dist = (src_i - src_j).norm(), src_ik_dist = (src_i - src_k).norm(), src_jk_dist = (src_j - src_k).norm();
            float tgt_ij_dist = (tgt_i - tgt_j).norm(), tgt_ik_dist = (tgt_i - tgt_k).norm(), tgt_jk_dist = (tgt_j - tgt_k).norm();
            float scale = 0.95;
            // Check if the distances between the points are within translation_resolution
            if (src_ij_dist < tgt_ij_dist * scale || tgt_ij_dist < src_ij_dist * scale ||
                src_ik_dist < tgt_ik_dist * scale || tgt_ik_dist < src_ik_dist * scale ||
                src_jk_dist < tgt_jk_dist * scale || tgt_jk_dist < src_jk_dist * scale)
            {
                continue;
            }
            Eigen::Matrix3Xd P(3, 3), Q(3, 3);
            P.col(0) = src_i; P.col(1) = src_j; P.col(2) = src_k;
            Q.col(0) = tgt_i; Q.col(1) = tgt_j; Q.col(2) = tgt_k;

            Eigen::Matrix4d transform_candidate = gtsam::svdSE3(P, Q);
            int inliers = 0;
            for (int i = 0; i < num_points; ++i) {
                Eigen::Vector3d src = src_points[associations(i, 0)];
                Eigen::Vector3d tgt = tgt_points[associations(i, 1)];
                Eigen::Vector3d transformed_src = (transform_candidate * src.homogeneous()).head<3>();
                if ((transformed_src - tgt).norm() < params.inlier_threshold) {
                    ++inliers;
                }
            }

#pragma omp critical
            {
                if (inliers > best_inliers && inliers >= params.min_inliers) {
                    best_inliers = inliers;
                    best_transform = transform_candidate;
                }
            }
        }
        return best_transform;
    }


    void solve(const std::vector<GraphVertex::Ptr> &src_nodes, const std::vector<GraphVertex::Ptr> &tgt_nodes,
                       const Association &A, FRGresult &result) {

        // RANSAC
        RansacParams params;
        params.max_iterations = config::ransac_max_iterations;
        params.min_inliers = 3;
        params.inlier_threshold = config::ransac_inlier_threshold;
        params.inliers_to_end = config::ransac_inliers_to_end;

        std::vector<Eigen::Vector3d> src_points, tgt_points;
        for (int i = 0; i < src_nodes.size(); ++i) {
            src_points.push_back(src_nodes[i]->centroid);
        }
        for (int i = 0; i < tgt_nodes.size(); ++i) {
            tgt_points.push_back(tgt_nodes[i]->centroid);
        }

        robot_utils::TicToc timer;
        Eigen::Matrix4d tf = ransac_registration(src_points, tgt_points, A, params);
        result.tf_solver_time = timer.toc();
        result.tf = tf;
    }
}