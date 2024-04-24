//
// Created by Zhijian QIAO on 23-5-24.
//

#ifndef SRC_PMC_SOLVER_H
#define SRC_PMC_SOLVER_H

#include "graph.h"
/**
 * A facade to the Parallel Maximum Clique (PMC) library.
 *  // 1. k-core pruning
    // 2. neigh-core pruning/ordering
    // 3. dynamic coloring bounds/sort
    // R. A. Rossi, D. F. Gleich, and A. H. Gebremedhin, “Parallel Maximum Clique Algorithms with Applications to Network Analysis,” SIAM J. Sci. Comput., vol. 37, no. 5, pp. C589–C616, Jan. 2015.
 * For details about PMC, please refer to:
 * https://github.com/ryanrossi/pmc
 * and
 * Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa Patwary, A Fast Parallel
 * Maximum Clique Algorithm for Large Sparse Graphs and Temporal Strong Components, arXiv preprint
 * 1302.6256, 2013.
 */

namespace clique_solver {
    class MaxCliqueSolver {
    public:
        /**
         * Enum representing the solver algorithm to use
         */
        enum class CLIQUE_SOLVER_MODE {
            PMC_EXACT = 0,
            PMC_HEU = 1,
            KCORE_HEU = 2,
        };

        /**
         * Parameter struct for MaxCliqueSolver
         */
        struct Params {

            /**
             * Algorithm used for finding max clique.
             */
            CLIQUE_SOLVER_MODE solver_mode = CLIQUE_SOLVER_MODE::PMC_EXACT;

            /**
             * \deprecated Use solver_mode instead
             * Set this to false to enable heuristic-only max clique finding.
             */
            bool solve_exactly = true;

            /**
             * The threshold ratio for determining whether to skip max clique and go straightly to
             * GNC rotation estimation. Set this to 1 to always use exact max clique selection, 0 to always
             * skip exact max clique selection.
             */
            double kcore_heuristic_threshold = 0.5;

            /**
             * Time limit on running the solver.
             */
            double time_limit = 3600;
        };

        MaxCliqueSolver() = default;

        MaxCliqueSolver(Params params) : params_(params) {};

        /**
         * Find the maximum clique within the graph provided. By maximum clique, it means the clique of
         * the largest size in an undirected graph.
         * @param graph
         * @return a vector of indices of cliques
         */
        std::vector<int> findMaxClique(Graph graph, int lower_bound = 0);

    private:
        Graph graph_;
        Params params_;
    };
}

#endif //SRC_PMC_SOLVER_H
