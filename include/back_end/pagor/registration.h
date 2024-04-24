/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_REGLIB_H
#define SRC_REGISTRATION_H

#include "clique_solver/pyclipper.h"
#include "back_end/teaser/registration.h"
#include "utils/opt_utils.h"
#include "front_end/graph_vertex.h"

namespace pagor {
    class PyramidRegistrationSolver : public teaser::RobustRegistrationSolver {
    public:
        PyramidRegistrationSolver(const teaser::RobustRegistrationSolver::Params &params, int num_graphs) : RobustRegistrationSolver(
                params) {
            num_graphs_ = num_graphs;
            // initialize the inlier_graphs_ and max_cliques_
            for (int i = 0; i < num_graphs_; ++i) {
                clique_solver::Graph graph;
                graph.clear();
                if (params.inlier_selection_mode == teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::CLIPPER ||
                    params.inlier_selection_mode == teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PYCLIPPER){
                    graph.setType(true);
                }
                else{
                    graph.setType(false);
                }
                inlier_graphs_.push_back(graph);
                max_cliques_.push_back(std::vector<int>());
            }
        }

        teaser::RegistrationSolution solve(const std::vector<clique_solver::GraphVertex::Ptr>& v1,
                                           const std::vector<clique_solver::GraphVertex::Ptr>& v2,
                                           const clique_solver::Association& A);

        void solveTransformGncTls(const std::vector<clique_solver::GraphVertex::Ptr>& v1, const std::vector<clique_solver::GraphVertex::Ptr>& v2, const std::vector<int>& max_clique, const int level, Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity());

        void solveTransformSVD(const std::vector<clique_solver::GraphVertex::Ptr>& v1, const std::vector<clique_solver::GraphVertex::Ptr>& v2, const std::vector<int>& max_clique, const int level);

        std::vector<int> solveTransformTeaser(const std::vector<clique_solver::GraphVertex::Ptr>& v1, const std::vector<clique_solver::GraphVertex::Ptr>& v2, const std::vector<int>& max_clique, const int level);

        void solveTransformGMM(const std::vector<g3reg::QuadricFeature::Ptr>& v1, const std::vector<g3reg::QuadricFeature::Ptr>& v2, const std::vector<int>& max_clique, const int level, Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity());

        void buildGraphs(const std::vector<clique_solver::GraphVertex::Ptr> &v1,
                        const std::vector<clique_solver::GraphVertex::Ptr> &v2);

        void solveMaxClique();

        void filterGraph(int level, const int max_clique_size);

        void setQuadricFeatures(const std::vector<g3reg::QuadricFeature::Ptr > &src_features,
                                 const std::vector<g3reg::QuadricFeature::Ptr > &dst_features);

    protected:
        int num_graphs_ = 1;
        size_t num_corr_ = 0;
        Eigen::MatrixXd pyramid_inliers_weight_;
        std::vector<clique_solver::Graph> inlier_graphs_;
        std::vector<std::vector<int>> max_cliques_;
        clique_solver::Association A_;
        std::vector<g3reg::QuadricFeature::Ptr > src_features_, dst_features_;
    };
}

#endif //SRC_REGLIB_H
