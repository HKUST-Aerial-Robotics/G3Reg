/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "clique_solver/pyclipper.h"

using namespace clique_solver;

namespace clipper {

    PyCLIPPER::PyCLIPPER(const std::vector<clique_solver::Graph> &graphs, const Params &params)
        :CLIPPER(params){
        // initialize the CLIPPERPyramid solver
        num_graphs_ = graphs.size();
        vM_.resize(num_graphs_);
        vC_.resize(num_graphs_);
        for (size_t i = 0; i < num_graphs_; ++i) {
            vM_[i] = graphs[i].affinity();
            vC_[i] = graphs[i].constraint();
        }
    }

    // ----------------------------------------------------------------------------

    void PyCLIPPER::solve(const Eigen::VectorXd &_u0) {
        Eigen::VectorXd u0;
        if (_u0.size() == 0) {
            u0 = randvec(vM_[0].cols());
        } else {
            u0 = _u0;
        }
        findDenseClique(u0);
    }

    void PyCLIPPER::findDenseClique(const Eigen::VectorXd &u0) {
        solns_.resize(num_graphs_);

        // Initialization

        const size_t n = vM_[0].cols();
        const Eigen::VectorXd ones = Eigen::VectorXd::Ones(n);

        // initialize memory
        Eigen::VectorXd gradF(n);
        Eigen::VectorXd gradFnew(n);
        Eigen::VectorXd u(n);
        Eigen::VectorXd unew(n);
        Eigen::VectorXd Mu(n);
        Eigen::VectorXd num(n);
        Eigen::VectorXd den(n);

        // one step of power method to have a good scaling of u
        if (params_.rescale_u0) {
            u = vM_[0].selfadjointView<Eigen::Upper>() * u0 + u0;
        } else {
            u = u0;
        }
        u /= u.norm();
        // initial value of d
        double d = 0; // zero if there are no active constraints
        Eigen::VectorXd Cbu = ones * u.sum() - vC_[0].selfadjointView<Eigen::Upper>() * u - u;
        const Eigen::VectorXi idxD = ((Cbu.array() > params_.eps) && (u.array() > params_.eps)).cast<int>();
        if (idxD.sum() > 0) {
            Mu = vM_[0].selfadjointView<Eigen::Upper>() * u + u;
            num = selectFromIndicator(Mu, idxD);
            den = selectFromIndicator(Cbu, idxD);
            d = (num.array() / den.array()).mean();
        }

        // Orthogonal projected gradient ascent with homotopy

        double F = 0; // objective value

        size_t i, j, k; // iteration counters
        size_t relax = 0; // relaxation counter
        for (i = 0; i < params_.maxoliters; ++i) {

            const SpAffinity &M_ = vM_[relax];
            const SpConstraint &C_ = vC_[relax];

            gradF = (1 + d) * u - d * ones * u.sum() + M_.selfadjointView<Eigen::Upper>() * u +
                    C_.selfadjointView<Eigen::Upper>() * u * d;
            F = u.dot(gradF); // current objective value

            // Orthogonal projected gradient ascent
            for (j = 0; j < params_.maxiniters; ++j) {
                double alpha = 1;

                // Backtracking line search on gradient ascent
                double Fnew = 0, deltaF = 0;
                for (k = 0; k < params_.maxlsiters; ++k) {
                    unew = u + alpha * gradF;                     // gradient step
                    unew = unew.cwiseMax(0);                      // project onto positive orthant
                    unew.normalize();                             // project onto S^n
                    gradFnew = (1 + d) * unew // because M/C is missing identity on diagonal
                               - d * ones * unew.sum()
                               + M_.selfadjointView<Eigen::Upper>() * unew
                               + C_.selfadjointView<Eigen::Upper>() * unew * d;
                    Fnew = unew.dot(gradFnew);                    // new objective value after step

                    deltaF = Fnew - F;                            // change in objective value

                    if (deltaF < -params_.eps) {
                        // objective value decreased---we need to backtrack, so reduce step size
                        alpha = alpha * params_.beta;
                    } else {
                        break; // obj value increased, stop line search
                    }
                }
                const double deltau = (unew - u).norm();

                // update values
                F = Fnew;
                u = unew;
                gradF = gradFnew;

                // check if desired accuracy has been reached by gradient ascent
                if (deltau < params_.tol_u || std::abs(deltaF) < params_.tol_F) break;
            }

            //
            // Increase d
            //

            Cbu = ones * u.sum() - C_.selfadjointView<Eigen::Upper>() * u - u;
            const Eigen::VectorXi idxD = ((Cbu.array() > params_.eps) && (u.array() > params_.eps)).cast<int>();
            if (idxD.sum() > 0) {
                Mu = M_.selfadjointView<Eigen::Upper>() * u + u;
                num = selectFromIndicator(Mu, idxD);
                den = selectFromIndicator(Cbu, idxD);
                const double deltad = (num.array() / den.array()).abs().mean();

                d += deltad;

            } else {
                transform2Solution(u, F, i, relax);
                relax++;
                if (relax == vM_.size())
                    break;
            }
        }
    }

    void PyCLIPPER::transform2Solution(const Eigen::VectorXd &u, double F, int i, const size_t &relax) {

        // estimate cluster size using largest eigenvalue
        const int omega = std::round(F);

        // extract indices of nodes in identified dense cluster
        std::vector<int> I = findIndicesOfkLargest(u, omega);

        // set solution
        Solution soln; ///< solution information from CLIPPERPyramid dense clique solver
        soln.ifinal = i;
        std::swap(soln.nodes, I);
        soln.u = u;
        soln.score = F;
        solns_[relax] = soln;
    }

}