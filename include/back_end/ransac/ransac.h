/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_RANSAC_H
#define SRC_RANSAC_H

#include "utils/evaluation.h"
#include "utils/config.h"
#include "front_end/graph_vertex.h"

namespace ransac {

    class RansacParams {
    public:
        int max_iterations;
        double inlier_threshold;
        int min_inliers;
        double inliers_to_end;

        RansacParams() {
            max_iterations = 1000;
            inlier_threshold = 0.6;
            min_inliers = 0;
            inliers_to_end = 0.5;
        }
    };

    void solve(const std::vector<clique_solver::GraphVertex::Ptr> &src_nodes,
               const std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes,
               const clique_solver::Association &A, FRGresult &result);
}

#endif //SRC_RANSAC_H
