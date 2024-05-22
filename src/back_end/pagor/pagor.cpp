/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "back_end/pagor/pagor.h"
#include "back_end/pagor/registration.h"
#include "front_end/gem/downsample.h"
#include "back_end/pagor/geo_verify.h"
#include <chrono>

using namespace std;
using namespace clique_solver;

namespace pagor {

    void countInliers(const teaser::RegistrationSolution &solution, const Eigen::Matrix4d &tf,
                      const clique_solver::Association &A, g3reg::EllipsoidMatcher &matcher, FRGresult &result) {
        if (matcher.getSrcEllipsoids().size() > 0) {
            // find the best transformation's index
            int best_tf_idx = 0;
            for (int i = 0; i < solution.candidates.size(); ++i) {
                if ((solution.candidates[i] - tf).norm() < 1e-3) {
                    best_tf_idx = i;
                    break;
                }
            }

            std::vector<std::pair<int, int>> inliers_vec;
            for (int i = 0; i < solution.inliers.cols(); ++i) {
                if (solution.inliers(best_tf_idx, i)) {
                    int src_id = A(i, 0);
                    int tgt_id = A(i, 1);
                    if (matcher.getSrcEllipsoids()[src_id]->type() == FeatureType::Plane)
                        result.plane_inliers++;
                    else if (matcher.getSrcEllipsoids()[src_id]->type() == FeatureType::Line)
                        result.line_inliers++;
                    else if (matcher.getSrcEllipsoids()[src_id]->type() == FeatureType::Cluster)
                        result.cluster_inliers++;
                    inliers_vec.emplace_back(src_id, tgt_id);
                }
            }
            clique_solver::Association inliers = clique_solver::Association::Zero(inliers_vec.size(), 2);
            for (int i = 0; i < inliers_vec.size(); ++i) {
                inliers(i, 0) = inliers_vec[i].first;
                inliers(i, 1) = inliers_vec[i].second;
            }
            result.inliers = inliers;
            return;
        }
        result.inliers = clique_solver::Association::Zero(0, 2);
    }

    void solve(const std::vector<GraphVertex::Ptr> &src_nodes, const std::vector<GraphVertex::Ptr> &tgt_nodes,
               const Association &A, g3reg::EllipsoidMatcher &matcher, FRGresult &result) {
        if (A.rows() == 0) {
            result.valid = false;
            return;
        }

        // initialize the parameters
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = config::vertex_info.noise_bound_vec[0];
        pagor::PyramidRegistrationSolver solver(params, config::num_graphs);
        solver.setQuadricFeatures(matcher.getSrcEllipsoids(), matcher.getTgtEllipsoids());
        solver.solve(src_nodes, tgt_nodes, A);
        teaser::RegistrationSolution solution = std::move(solver.getSolution());

        robot_utils::TicToc verify_timer;
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        bool verify_valid = true;
        if (config::verify_mtd == "voxel" && matcher.getSrcVoxels().size() > 0 && matcher.getTgtVoxels().size() > 0) {
//            std::tie(verify_valid, tf) = GeometryVerify(matcher.getSrcVoxels(), matcher.getTgtVoxels(), solution.candidates);
            std::tie(verify_valid, tf) = GeometryVerifyNanoFlann(matcher.getSrcVoxels(), matcher.getTgtVoxels(),
                                                                 solution.candidates);
        } else { // default: use point cloud
            tf = GeometryVerify(matcher.getSrcPc(), matcher.getTgtPc(), solution.candidates);
        }
        double verify_time = verify_timer.toc();

        result.tf = tf;
        result.valid = solution.valid && verify_valid;
        result.clique_time = solution.clique_time;
        result.graph_time = solution.graph_time;
        result.tf_solver_time = solution.tf_solver_time;
        result.verify_time = verify_time;
        result.candidates = solution.candidates;
        countInliers(solution, tf, A, matcher, result);
    }
}

