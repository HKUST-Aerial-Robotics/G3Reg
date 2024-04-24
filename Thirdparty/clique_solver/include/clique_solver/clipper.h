/**
 * @file clipper.h
 * @brief CLIPPER data association framework
 * @author Parker Lusk <plusk@mit.edu>
 * @date 3 October 2020
 */

#pragma once

#include <tuple>
#include <Eigen/Dense>
#include <random>
#include <queue>
#include "graph.h"

namespace clipper {

    /**
     * @brief      CLIPPER parameters
     */
    struct Params {

        // \brief Basic gradient descent stopping criteria
        double tol_u = 1e-8; ///< stop when change in u < tol
        double tol_F = 1e-9; ///< stop when change in F < tol
        double tol_Fop = 1e-10; ///< stop when ||dFop|| < tol
        int maxiniters = 200; ///< max num of gradient ascent steps for each d
        int maxoliters = 1000; ///< max num of outer loop iterations to find d

        // \brief Line search parameters
        double beta = 0.25; ///< backtracking step size reduction, in (0, 1)
        int maxlsiters = 99; ///< maximum number of line search iters per grad step

        double eps = 1e-9; ///< numerical threshold around 0

        double affinityeps = 1e-4; ///< sparsity-promoting threshold for affinities

        bool rescale_u0 = true; ///< Rescale u0 using one power iteration. This
        ///< removes some randomness of the initial guess;
        ///< i.e., after one step of power method, random
        ///< u0's look similar.
    };

    /**
     * @brief      Data associated with a CLIPPER dense clique solution
     */
    class Solution {
        public:
        int ifinal; ///< number of outer iterations before convergence
        std::vector<int> nodes; ///< indices of graph vertices in dense clique
        Eigen::VectorXd u; ///< characteristic vector associated with graph
        double score; ///< value of objective function / largest eigenvalue
    };

    /**
     * @brief      Convenience class to use CLIPPER for data association.
     */
    class CLIPPER {
    public:
        CLIPPER() = default;

        CLIPPER(const clique_solver::Graph &graph, const Params &params);

        CLIPPER(const Params &params) : params_(params) {};

        virtual ~CLIPPER() {}

        virtual void solve(const Eigen::VectorXd &u0 = Eigen::VectorXd());

        const Solution &getSolution() const { return soln_; }

        clique_solver::Affinity getAffinityMatrix();

        clique_solver::Constraint getConstraintMatrix();

        /**
         * @brief      Skip using scorePairwiseConsistency and directly set the
         *             affinity and constraint matrices. Note that this function
         *             accepts sparse matrices. These matrices should be upper
         *             triangular and should not have diagonal values set.
         *
         * @param[in]  M     Affinity matrix
         * @param[in]  C     Constraint matrix
         */
        void setSparseMatrixData(const clique_solver::SpAffinity &M, const clique_solver::SpConstraint &C);

        void setParallelize(bool parallelize) { parallelize_ = parallelize; };

    protected:
        Params params_;

        bool parallelize_ = true; ///< should affinity calculation be parallelized

        // \brief Problem data from latest instance of data association
        Solution soln_; ///< solution information from CLIPPER dense clique solver
        clique_solver::SpAffinity M_; ///< affinity matrix (i.e., weighted consistency graph)
        clique_solver::SpConstraint C_; ///< constraint matrix (i.e., prevents forming links)

        /**
         * @brief      Identifies a dense clique of an undirected graph G from its
         *             weighted affinity matrix M while satisfying any active
         *             constraints in C (indicated with zeros).
         *
         *             If M is binary and C==M then CLIPPER returns a maximal clique.
         *
         *             This algorithm employs a projected gradient descent method to
         *             solve a symmetric rank-one nonnegative matrix approximation.
         *
         * @param[in]  M        Symmetric, non-negative nxn affinity matrix where
         *                      each element is in [0,1]. Nodes can also be weighted
         *                      between [0,1] (e.g., if there is a prior indication
         *                      that a node belongs to the desired cluster). In the
         *                      case that all nodes are equally likely to be in the
         *                      densest cluster/node weights should not be considered
         *                      set the diagonal of M to identity.
         * @param[in]  C        nxn binary constraint matrix. Active const. are 0.
         */
        void findDenseClique(const Eigen::VectorXd &u0);


        /**
         * @brief      Select the elements of a vector x given an indicator vector.
         *
         * @param[in]  x     Vector to select elements of
         * @param[in]  ind   The indicator vector
         *
         * @return     Vector of selected elements, with size <= x.size
         */
        Eigen::VectorXd selectFromIndicator(const Eigen::VectorXd &x,
                                            const Eigen::VectorXi &ind);

        /**
         * @brief      Produce an nx1 vector where each element is drawn from U[0, 1).
         *
         * @param[in]  n     Dimension of produced vector
         *
         * @return     Uniform random vector
         */
        Eigen::VectorXd randvec(size_t n);

        /**
         * @brief      Find indices of k largest elements of vector (similar to MATLAB find)
         *
         * @param[in]  x     Vector to find large elements in
         * @param[in]  k     How many of the largest elements to find (k > 0)
         *
         * @return     Indices of the largest elements in vector x
         */
        std::vector<int> findIndicesOfkLargest(const Eigen::VectorXd &x, int k);
    };

} // ns clipper