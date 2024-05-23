/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "back_end/pagor/registration.h"
#include <chrono>
#include <glog/logging.h>
#include "robot_utils/tic_toc.h"
#include "back_end/teaser/quatro.h"
#include "robot_utils/algorithms.h"

using namespace teaser;
using namespace g3reg;

namespace pagor {

    teaser::RegistrationSolution PyramidRegistrationSolver::
    solve(const std::vector<clique_solver::GraphVertex::Ptr> &src,
          const std::vector<clique_solver::GraphVertex::Ptr> &dst,
          const clique_solver::Association &A) {

        assert(scale_solver_ && rotation_solver_ && translation_solver_);

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        /**Steps to estimate T/R/s**/
        A_ = A; // MatrixX2d

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        buildGraphs(src, dst);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        solveMaxClique();
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

        // Update validity flag
        solution_.inliers = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_graphs_, A_.rows());
        solution_.candidates.resize(num_graphs_);
        for (int level = 0; level < num_graphs_; ++level) {
            const auto &max_clique = max_cliques_[level];
            bool same_clique =
                    level == 0 ? false : robot_utils::are_vectors_equal(max_cliques_[level - 1], max_cliques_[level]);
            if (same_clique) {
                solution_.inliers.row(level) = solution_.inliers.row(level - 1);
                solution_.candidates[level] = solution_.candidates[level - 1];
                continue;
            }
            if (config::tf_solver == "gmm_tls") {
                solveTransformSVD(src, dst, max_clique, level);
                solveTransformGMM(src_features_, dst_features_, max_clique, level, solution_.candidates[level]);
            } else if (config::tf_solver == "gnc") {
                solveTransformSVD(src, dst, max_clique, level);
                solveTransformGncTls(src, dst, max_clique, level, solution_.candidates[level]);
            } else if (config::tf_solver == "svd") {
                solveTransformSVD(src, dst, max_clique, level);
            } else if (config::tf_solver == "teaser" || config::tf_solver == "quatro") {
                solveTransformTeaser(src, dst, max_clique, level);
            } else {
                throw std::runtime_error("Unknown tf solver type");
            }
        }

        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        solution_.graph_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;
        solution_.clique_time = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0;
        solution_.tf_solver_time = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() / 1000.0;
        return solution_;
    }

    void PyramidRegistrationSolver::solveTransformGMM(const std::vector<g3reg::QuadricFeature::Ptr> &v1,
                                                      const std::vector<g3reg::QuadricFeature::Ptr> &v2,
                                                      const std::vector<int> &max_clique, const int level,
                                                      Eigen::Matrix4d T_init) {
        // when rotation inlier selection is enabled, we need to prune the rotation outliers. otherwise, we just use associations in max clique
        clique_solver::Association rotation_pruned_A = clique_solver::Association::Zero(max_clique.size(), 2);
        for (size_t i = 0; i < max_clique.size(); ++i) {
            rotation_pruned_A.row(i) = A_.row(max_clique[i]);
        }

        // Abort if max max_clique size <= 1
        if (max_clique.size() >= 3) {
            solution_.candidates[level] = gtsam::gncQuadricsSE3(v1, v2, rotation_pruned_A, T_init);
            solution_.valid = true;
            for (size_t i = 0; i < max_clique.size(); ++i) {
                solution_.inliers(level, max_clique[i]) = true;
            }
        } else {
            solution_.valid = false;
            solution_.candidates[level] = Eigen::Matrix4d::Identity();
        }
    }

    void PyramidRegistrationSolver::solveTransformGncTls(const std::vector<clique_solver::GraphVertex::Ptr> &src,
                                                         const std::vector<clique_solver::GraphVertex::Ptr> &dst,
                                                         const std::vector<int> &max_clique, const int level,
                                                         Eigen::Matrix4d T_init) {
        // when rotation inlier selection is enabled, we need to prune the rotation outliers. otherwise, we just use associations in max clique
        Eigen::Matrix3Xd rotation_pruned_src;
        Eigen::Matrix3Xd rotation_pruned_dst;
        rotation_pruned_src.resize(3, max_clique.size());
        rotation_pruned_dst.resize(3, max_clique.size());
        for (size_t i = 0; i < max_clique.size(); ++i) {
            rotation_pruned_src.col(i) = src.at(A_(max_clique[i], 0))->centroid;
            rotation_pruned_dst.col(i) = dst.at(A_(max_clique[i], 1))->centroid;
        }

        // Abort if max max_clique size <= 1
        if (max_clique.size() >= 3) {
            solution_.candidates[level] = gtsam::gncSE3(rotation_pruned_src, rotation_pruned_dst);
            for (size_t i = 0; i < max_clique.size(); ++i) {
                solution_.inliers(level, max_clique[i]) = true;
            }
            solution_.valid = true;
        } else {
            solution_.valid = false;
            solution_.candidates[level] = Eigen::Matrix4d::Identity();
        }
    }

    void PyramidRegistrationSolver::solveTransformSVD(const std::vector<clique_solver::GraphVertex::Ptr> &src,
                                                      const std::vector<clique_solver::GraphVertex::Ptr> &dst,
                                                      const std::vector<int> &max_clique, const int level) {
        // when rotation inlier selection is enabled, we need to prune the rotation outliers. otherwise, we just use associations in max clique
        Eigen::Matrix3Xd rotation_pruned_src;
        Eigen::Matrix3Xd rotation_pruned_dst;
        rotation_pruned_src.resize(3, max_clique.size());
        rotation_pruned_dst.resize(3, max_clique.size());
        for (size_t i = 0; i < max_clique.size(); ++i) {
            rotation_pruned_src.col(i) = src.at(A_(max_clique[i], 0))->centroid;
            rotation_pruned_dst.col(i) = dst.at(A_(max_clique[i], 1))->centroid;
        }

        // Abort if max max_clique size <= 1
        if (max_clique.size() >= 3) {
            solution_.candidates[level] = gtsam::svdSE3(rotation_pruned_src, rotation_pruned_dst);
            for (size_t i = 0; i < max_clique.size(); ++i) {
                solution_.inliers(level, max_clique[i]) = true;
            }
            solution_.valid = true;
        } else {
            solution_.valid = false;
            solution_.candidates[level] = Eigen::Matrix4d::Identity();
        }
    }

    std::vector<int>
    PyramidRegistrationSolver::solveTransformTeaser(const std::vector<clique_solver::GraphVertex::Ptr> &src,
                                                    const std::vector<clique_solver::GraphVertex::Ptr> &dst,
                                                    const std::vector<int> &max_clique, const int level) {
        reset(params_);
        if (max_clique.size() <= 1) {
            solution_.valid = false;
            solution_.candidates[level] = Eigen::Matrix4d::Identity();
            solution_.inliers.row(level).setZero();
            return max_clique;
        }

        // TIMs used for rotation estimation
        Eigen::Matrix3Xd pruned_src_tims;
        Eigen::Matrix3Xd pruned_dst_tims;
        // TIM maps for rotation estimation
        Eigen::Matrix<int, 2, Eigen::Dynamic> src_tims_map_rotation;
        Eigen::Matrix<int, 2, Eigen::Dynamic> dst_tims_map_rotation;

        // Calculate new measurements & TIMs based on max clique inliers
        if (params_.rotation_tim_graph == INLIER_GRAPH_FORMULATION::CHAIN) {
            // chain graph 知道正确的匹配关系（可能还含有一些外点）后，用chain graph来构造TIMS来估计R
            TEASER_DEBUG_INFO_MSG("Using chain graph for GNC rotation.");
            pruned_src_tims.resize(3, max_clique.size());
            pruned_dst_tims.resize(3, max_clique.size());
            src_tims_map_rotation.resize(2, max_clique.size());
            dst_tims_map_rotation.resize(2, max_clique.size());
            for (size_t i = 0; i < max_clique.size(); ++i) {
                const auto &root = max_clique[i];
                int leaf;
                if (i != max_clique.size() - 1) {
                    leaf = max_clique[i + 1];
                } else {
                    leaf = max_clique[0];
                }

                pruned_src_tims.col(i) = src.at(A_(leaf, 0))->centroid - src.at(A_(root, 0))->centroid;
                pruned_dst_tims.col(i) = dst.at(A_(leaf, 1))->centroid - dst.at(A_(root, 1))->centroid;

                // populate the TIMs map
                dst_tims_map_rotation(0, i) = leaf;
                dst_tims_map_rotation(1, i) = root;
                src_tims_map_rotation(0, i) = leaf;
                src_tims_map_rotation(1, i) = root;
            }
        } else {
            // complete graph
            TEASER_DEBUG_INFO_MSG("Using complete graph for GNC rotation.");
            // select the inlier measurements with max clique
            Eigen::Matrix3Xd src_inliers(3, max_clique.size());
            Eigen::Matrix3Xd dst_inliers(3, max_clique.size());
            for (size_t i = 0; i < max_clique.size(); ++i) {
                src_inliers.col(i) = src.at(A_(max_clique[i], 0))->centroid;
                dst_inliers.col(i) = dst.at(A_(max_clique[i], 1))->centroid;
            }
            // construct the TIMs
            pruned_dst_tims = RobustRegistrationSolver::computeTIMs(dst_inliers, &dst_tims_map_rotation);
            pruned_src_tims = RobustRegistrationSolver::computeTIMs(src_inliers, &src_tims_map_rotation);
        }

        // Remove scaling for rotation estimation
        pruned_dst_tims *= (1 / solution_.scale);

        // Update GNC rotation solver's noise bound with the new information
        // Note: this implicitly assumes that rotation_solver_'s noise bound
        // is set to the original noise bound of the measurements.
        auto params = rotation_solver_->getParams();
        params.noise_bound *= (2 / solution_.scale);
        rotation_solver_->setParams(params);

        // Solve for rotation
        TEASER_DEBUG_INFO_MSG("Starting rotation solver.");
        if (config::tf_solver == "quatro") {
            quatro::QuatroSolver solver(params_);
            solver.solveForRotation(pruned_src_tims, pruned_dst_tims);
            rotation_inliers_mask_ = solver.getRotationInliersMask();
            solution_.rotation = solver.getRotationSolution();
        } else {
            solveForRotation(pruned_src_tims, pruned_dst_tims);
        }
        TEASER_DEBUG_INFO_MSG("Rotation estimation complete.");

        // Save indices of inlier TIMs from GNC rotation estimation
        if (params_.rotation_inlier_selection) {
            for (size_t i = 0; i < rotation_inliers_mask_.cols(); ++i) {
                int dst_idx = i - 1 < 0 ? rotation_inliers_mask_.cols() - 1 : i - 1;
                if (rotation_inliers_mask_(0, dst_idx) && rotation_inliers_mask_(0, i)) {
                    rotation_inliers_.emplace_back(i);
                }
            }
        } else {
            for (size_t i = 0; i < rotation_inliers_mask_.cols(); ++i) {
                if (rotation_inliers_mask_[i]) {
                    rotation_inliers_.emplace_back(i);
                }
            }
        }

        // when rotation inlier selection is enabled, we need to prune the rotation outliers. otherwise, we just use associations in max clique
        Eigen::Matrix3Xd rotation_pruned_src;
        Eigen::Matrix3Xd rotation_pruned_dst;
        if (params_.rotation_inlier_selection && rotation_inliers_.size() > 0) {
            rotation_pruned_src.resize(3, rotation_inliers_.size());
            rotation_pruned_dst.resize(3, rotation_inliers_.size());
            for (size_t i = 0; i < rotation_inliers_.size(); ++i) {
                rotation_pruned_src.col(i) = src.at(A_(max_clique[rotation_inliers_[i]], 0))->centroid;
                rotation_pruned_dst.col(i) = dst.at(A_(max_clique[rotation_inliers_[i]], 1))->centroid;
            }
        } else {
            rotation_pruned_src.resize(3, max_clique.size());
            rotation_pruned_dst.resize(3, max_clique.size());
            for (size_t i = 0; i < max_clique.size(); ++i) {
                rotation_pruned_src.col(i) = src.at(A_(max_clique[i], 0))->centroid;
                rotation_pruned_dst.col(i) = dst.at(A_(max_clique[i], 1))->centroid;
            }
        }

        // Solve for translation
        TEASER_DEBUG_INFO_MSG("Starting translation solver.");
        solveForTranslation(solution_.scale * solution_.rotation * rotation_pruned_src,
                            rotation_pruned_dst);
        TEASER_DEBUG_INFO_MSG("Translation estimation complete.");
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

        // Find the final inliers
        translation_inliers_ = robot_utils::findNonzero<bool>(translation_inliers_mask_);

        // Update validity flag
        solution_.valid = true;
        solution_.candidates[level] = Eigen::Matrix4d::Identity();
        solution_.candidates[level].block<3, 3>(0, 0) = solution_.rotation;
        solution_.candidates[level].block<3, 1>(0, 3) = solution_.translation;
        // Update final inliers
        std::vector<int> final_clique;
        final_clique.reserve(max_clique.size());
        for (const auto &inlier: translation_inliers_) {
            int idx = params_.rotation_inlier_selection && rotation_inliers_.size() > 0 ?
                      rotation_inliers_[inlier] : inlier;
            solution_.inliers(level, max_clique[idx]) = true;
            final_clique.emplace_back(max_clique[idx]);
        }
        return final_clique;
    }


    void PyramidRegistrationSolver::filterGraph(int level, const int max_clique_size) {
        if (level >= inlier_graphs_.size())
            return;
        clique_solver::Graph &graph = inlier_graphs_[level];
        if (max_clique_size > 0 && graph.numVertices() > max_clique_size) {
            graph.pruneGraph(max_clique_size);
        }
    }

    void PyramidRegistrationSolver::solveMaxClique() {
        // Handle deprecated params
        if (!params_.use_max_clique) {
            TEASER_DEBUG_INFO_MSG(
                    "Using deprecated param field use_max_clique. Switch to inlier_selection_mode instead.");
            params_.inlier_selection_mode = INLIER_SELECTION_MODE::NONE;
        }
        if (!params_.max_clique_exact_solution) {
            TEASER_DEBUG_INFO_MSG("Using deprecated param field max_clique_exact_solution. Switch to "
                                  "inlier_selection_mode instead.");
            params_.inlier_selection_mode = INLIER_SELECTION_MODE::PMC_HEU;
        }
        // Calculate Maximum Clique
        if (params_.inlier_selection_mode != INLIER_SELECTION_MODE::NONE) {
            if (params_.inlier_selection_mode == INLIER_SELECTION_MODE::PMC_EXACT) {
                clique_solver::MaxCliqueSolver::Params clique_params;
                clique_params.solver_mode = clique_solver::MaxCliqueSolver::CLIQUE_SOLVER_MODE::PMC_EXACT;
                clique_params.time_limit = params_.max_clique_time_limit;
                clique_params.kcore_heuristic_threshold = params_.kcore_heuristic_threshold;
                int prune_level = 0;
                for (int level = 0; level < num_graphs_; ++level) {
                    clique_solver::MaxCliqueSolver mac_solver(clique_params);
                    max_cliques_[level] = mac_solver.findMaxClique(inlier_graphs_[level], prune_level);
                    prune_level = config::grad_pmc ? max_cliques_[level].size() : 0;
                }
            }
            for (int level = 0; level < num_graphs_; ++level) {
                auto &clique = max_cliques_[level];
                std::sort(clique.begin(), clique.end());
            }
        } else {
            // not using clique filtering is equivalent to saying all measurements are in the max clique
            for (int level = 0; level < num_graphs_; ++level) {
                auto &clique = max_cliques_[level];
                clique.reserve(num_corr_);
                for (size_t i = 0; i < num_corr_; ++i) {
                    clique.push_back(i);
                }
            }
        }
    }


    void PyramidRegistrationSolver::buildGraphs(const std::vector<clique_solver::GraphVertex::Ptr> &v1,
                                                const std::vector<clique_solver::GraphVertex::Ptr> &v2) {
        num_corr_ = A_.rows();
        int num_tims = num_corr_ * (num_corr_ - 1) / 2;

        for (int level = 0; level < num_graphs_; ++level) {
            inlier_graphs_[level].populateVertices(num_corr_);
        }

        robot_utils::TicToc t_graph;
#pragma omp parallel for default(none) shared(num_corr_, num_tims, v1, v2, inlier_graphs_, A_)
        for (size_t k = 0; k < num_tims; ++k) {
            size_t i, j;
            std::tie(i, j) = clique_solver::k2ij(k, num_corr_);
            const auto &weights = (*v1[A_(j, 0)] - *v1[A_(i, 0)])->consistent(*(*v2[A_(j, 1)] - *v2[A_(i, 1)]));
            for (int level = 0; level < num_graphs_; ++level) {
                if (weights(level) > 0.0) {
#pragma omp critical
                    {
                        inlier_graphs_[level].addEdge(i, j);
                    }
                }
            }
        }

        // We assume no scale difference between the two vectors of points.
        solution_.scale = 1.0;
    }

    void PyramidRegistrationSolver::setQuadricFeatures(const std::vector<g3reg::QuadricFeature::Ptr> &src_features,
                                                       const std::vector<g3reg::QuadricFeature::Ptr> &dst_features) {
        src_features_ = std::move(src_features);
        dst_features_ = std::move(dst_features);
    }
}
