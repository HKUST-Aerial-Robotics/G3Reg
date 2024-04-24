/**
 ============================================================================
 Name        : Parallel Maximum Clique (PMC) Library
 Author      : Ryan A. Rossi   (rrossi@purdue.edu)
 Description : A general high-performance parallel framework for computing
               maximum cliques. The library is designed to be fast for large
               sparse graphs.

 Copyright (C) 2012-2013, Ryan A. Rossi, All rights reserved.

 Please cite the following paper if used:
   Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa
   Patwary, A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs
   and Temporal Strong Components, arXiv preprint 1302.6256, 2013.

 See http://ryanrossi.com/pmc for more information.
 ============================================================================
 */

#ifndef PMC_HEU_H_
#define PMC_HEU_H_

#include "pmc_headers.h"
#include "pmc_graph.h"
#include "pmc_utils.h"
#include "pmc_input.h"
#include "pmc_vertex.h"
#include <algorithm>

namespace pmc {

    class pmc_heu {
        public:
            vector<int>* E;
            vector<long long>* V;
            vector<int>* K;
            vector<int>* order;
            vector<int>* degree;
            double sec;
            int ub, lb;
            string strat;

            int num_threads;

            pmc_heu(pmc_graph& G, input& params) {
                K = G.get_kcores();
                order = G.get_kcore_ordering();
                ub = params.ub;
                lb = params.lb;
                strat = params.heu_strat;
                num_threads = params.threads;
                initialize();
            }

            pmc_heu(pmc_graph& G, int tmp_ub) {
                K = G.get_kcores();
                order = G.get_kcore_ordering();
                ub = tmp_ub;
                strat = "kcore";
                initialize();
            }

            inline void initialize() {
                sec = get_time();
                srand (time(NULL));
            };

            int strategy(vector<int>& P);
            void set_strategy(string s) { strat = s; }
            int compute_heuristic(int v);

            static bool desc_heur(Vertex v,  Vertex u) {
                return (v.get_bound() > u.get_bound());
            }

            static bool incr_heur(Vertex v,  Vertex u) {
                return (v.get_bound() < u.get_bound());
            }

            int search(pmc_graph& graph, vector<int>& C_max);
            int search_cores(pmc_graph& graph, vector<int>& C_max, int lb);
            int search_bounds(pmc_graph& graph, vector<int>& C_max);

            inline void branch(vector<Vertex>& P, int sz,
                    int& mc, vector<int>& C, vector<short>& ind);

            inline void print_info(vector<int> C_max);
    };
};
#endif
