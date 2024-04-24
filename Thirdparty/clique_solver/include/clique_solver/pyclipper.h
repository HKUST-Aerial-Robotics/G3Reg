/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_PYCLIPPER_H
#define SRC_PYCLIPPER_H

#include "clipper.h"

namespace clipper{
    class PyCLIPPER : public CLIPPER {

    public:

        PyCLIPPER() = default;

        PyCLIPPER(const std::vector<clique_solver::Graph> &graphs, const Params &params);

        void solve(const Eigen::VectorXd &u0 = Eigen::VectorXd());

        void findDenseClique(const Eigen::VectorXd &u0);

        void transform2Solution(const Eigen::VectorXd &u, double F, int i, const size_t &relax);

        const std::vector<Solution>& getSolutions() { return solns_; }

        //~PyCLIPPER(){
        //    CLIPPER::~CLIPPER();
        //    solns_.clear();
        //    vM_.clear();
        //    vC_.clear();
        //}

    private:
        std::vector<Solution> solns_; ///< solution information from CLIPPERPyramid clique solver
        std::vector<clique_solver::SpAffinity> vM_; ///< affinity matrix vector
        std::vector<clique_solver::SpConstraint> vC_; ///< constraint matrix vector
        int num_graphs_;
    };
}

#endif //SRC_PYCLIPPER_H
