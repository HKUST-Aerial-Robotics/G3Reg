/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
//

#ifndef SRC_PAGOR_H
#define SRC_PAGOR_H

#include "utils/config.h"
#include "front_end/gem/ellipsoid.h"

namespace pagor{

    teaser::RobustRegistrationSolver::Params getParams();

    void solve(const std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, const std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes,
               const clique_solver::Association &A, g3reg::EllipsoidMatcher &matcher, FRGresult& result);
}

#endif //SRC_PAGOR_H
