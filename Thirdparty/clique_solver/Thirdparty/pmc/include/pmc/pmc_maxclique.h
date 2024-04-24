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

#ifndef PMC_MAXCLIQUE_H_
#define PMC_MAXCLIQUE_H_

#include <cstddef>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include "pmc_headers.h"
#include "pmc_utils.h"
#include "pmc_graph.h"
#include "pmc_input.h"
#include "pmc_vertex.h"

using namespace std;

namespace pmc {

    class pmc_maxclique {
        public:
            vector<int>* edges;
            vector<long long>* vertices;
            vector<int>* bound;
            vector<int>* order;
            vector<int>* degree;
            int param_ub;
            int ub;
            int lb;
            double time_limit;
            double sec;
            double wait_time;
            bool not_reached_ub;
            bool time_expired_msg;
            bool decr_order;

            string vertex_ordering;
            int edge_ordering;
            int style_bounds;
            int style_dynamic_bounds;

            int num_threads;

            void initialize() {
                vertex_ordering = "kcore";
                edge_ordering = 0;
                style_bounds = 0;
                style_dynamic_bounds = 0;
                not_reached_ub = true;
                time_expired_msg = true;
                decr_order = false;
            }

            void setup_bounds(input& params) {
                lb = params.lb;
                ub = params.ub;
                param_ub = params.param_ub;
                if (param_ub == 0)
                    param_ub = ub;
                time_limit = params.time_limit;
                wait_time = params.remove_time;
                sec = get_time();

                num_threads = params.threads;
            }


            pmc_maxclique(pmc_graph& G, input& params) {
                bound = G.get_kcores();
                order = G.get_kcore_ordering();
                setup_bounds(params);
                initialize();
                vertex_ordering = params.vertex_search_order;
                decr_order = params.decreasing_order;
            }

            ~pmc_maxclique() {};

            int search(pmc_graph& G, vector<int>& sol);

            void branch(
                    vector<Vertex> &P,
                    vector<short>& ind,
                    vector<int>& C,
                    vector<int>& C_max,
                    int* &pruned,
                    int& mc);


            int search_dense(pmc_graph& G, vector<int>& sol);

            void branch_dense(
                    vector<Vertex> &P,
                    vector<short>& ind,
                    vector<int>& C,
                    vector<int>& C_max,
                    int* &pruned,
                    int& mc,
                    vector<vector<bool>> &adj);

    };
};

#endif
