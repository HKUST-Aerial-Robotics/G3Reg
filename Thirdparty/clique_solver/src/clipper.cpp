/**
 * @file clipper.cpp
 * @brief CLIPPER data association framework
 * @author Parker Lusk <plusk@mit.edu>
 * @date 19 March 2022
 */

#include <iostream>
#include "clique_solver/clipper.h"

using namespace clique_solver;
namespace clipper {

    CLIPPER::CLIPPER(const clique_solver::Graph &graph, const Params &params)
            : params_(params) {
        setSparseMatrixData(graph.affinity(), graph.constraint());
    }

    // ----------------------------------------------------------------------------

    void CLIPPER::solve(const Eigen::VectorXd &_u0) {
        Eigen::VectorXd u0;
        if (_u0.size() == 0) {
            u0 = randvec(M_.cols());
        } else {
            u0 = _u0;
        }
        findDenseClique(u0);
    }

    // ----------------------------------------------------------------------------
    // Private Methods
    // ----------------------------------------------------------------------------

    void CLIPPER::findDenseClique(const Eigen::VectorXd &u0) {

        //
        // Initialization
        //

        const size_t n = M_.cols();
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
            u = M_.selfadjointView<Eigen::Upper>() * u0 + u0;
        } else {
            u = u0;
        }
        u /= u.norm();

        // initial value of d
        double d = 0; // zero if there are no active constraints
        Eigen::VectorXd Cbu = ones * u.sum() - C_.selfadjointView<Eigen::Upper>() * u - u;
        const Eigen::VectorXi idxD = ((Cbu.array() > params_.eps) && (u.array() > params_.eps)).cast<int>();
        if (idxD.sum() > 0) {
            Mu = M_.selfadjointView<Eigen::Upper>() * u + u;
            num = selectFromIndicator(Mu, idxD);
            den = selectFromIndicator(Cbu, idxD);
            d = (num.array() / den.array()).mean();
        }

        //
        // Orthogonal projected gradient ascent with homotopy
        //

        double F = 0; // objective value

        size_t i, j, k; // iteration counters
        for (i = 0; i < params_.maxoliters; ++i) {
            gradF = (1 + d) * u - d * ones * u.sum() + M_.selfadjointView<Eigen::Upper>() * u +
                    C_.selfadjointView<Eigen::Upper>() * u * d;
            F = u.dot(gradF); // current objective value

            //
            // Orthogonal projected gradient ascent
            //

            for (j = 0; j < params_.maxiniters; ++j) {
                double alpha = 1;

                //
                // Backtracking line search on gradient ascent
                //

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
                break;
            }
        }

        // Generate output

        // estimate cluster size using largest eigenvalue
        const int omega = std::round(F);

        // extract indices of nodes in identified dense cluster
        std::vector<int> I = findIndicesOfkLargest(u, omega);

        // set solution
        soln_.ifinal = i;
        std::swap(soln_.nodes, I);
        soln_.u.swap(u);
        soln_.score = F;
    }

    // ----------------------------------------------------------------------------

    Affinity CLIPPER::getAffinityMatrix() {
        Affinity M = SpAffinity(M_.selfadjointView<Eigen::Upper>())
                     + Affinity::Identity(M_.rows(), M_.cols());
        return M;
    }

    // ----------------------------------------------------------------------------

    Constraint CLIPPER::getConstraintMatrix() {
        Constraint C = SpConstraint(C_.selfadjointView<Eigen::Upper>())
                       + Constraint::Identity(C_.rows(), C_.cols());
        return C;
    }

    // ----------------------------------------------------------------------------

    void CLIPPER::setSparseMatrixData(const SpAffinity &M, const SpConstraint &C) {
        M_ = M;
        C_ = C;
    }


    // ----------------------------------------------------------------------------

    Eigen::VectorXd CLIPPER::selectFromIndicator(const Eigen::VectorXd &x,
                                        const Eigen::VectorXi &ind) {
        Eigen::VectorXd y(ind.sum());
        size_t idx = 0;
        for (size_t i = 0; i < x.size(); ++i) {
            if (ind[i]) {
                y[idx++] = x[i];
            }
        }
        return y;
    }

    Eigen::VectorXd CLIPPER::randvec(size_t n) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0, 1);

        return Eigen::VectorXd::NullaryExpr(n, 1, [&]() { return dis(gen); });
    }

    // ----------------------------------------------------------------------------

    std::vector<int> CLIPPER::findIndicesOfkLargest(const Eigen::VectorXd &x, int k) {
        using T = std::pair<double, int>; // pair value to be compared and index
        if (k < 1) return {}; // invalid input
        // n.b., the top of this queue is smallest element
        std::priority_queue<T, std::vector<T>, std::greater<T>> q;
        for (size_t i = 0; i < x.rows(); ++i) {
            if (q.size() < k) {
                q.push({x(i), i});
            } else if (q.top().first < x(i)) {
                q.pop();
                q.push({x(i), i});
            }
        }

        std::vector<int> indices(k);
        for (size_t i = 0; i < k; ++i) {
            indices[k - i - 1] = q.top().second;
            q.pop();
        }

        return indices;
    }

} // ns clipper