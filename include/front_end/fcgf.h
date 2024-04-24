/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_FCGF_H
#define SRC_FCGF_H

#include "front_end/gem/ellipsoid.h"

namespace fcgf{
    clique_solver::Association matching(std::tuple<int, int, int> pair_info,
                                        std::vector<clique_solver::GraphVertex::Ptr> &src_nodes,
                                        std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes);
}

#endif //SRC_FCGF_H
