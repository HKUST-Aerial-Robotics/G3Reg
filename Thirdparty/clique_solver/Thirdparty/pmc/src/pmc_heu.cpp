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

#include "pmc/pmc_debug_utils.h"
#include "pmc/pmc_heu.h"

using namespace pmc;
using namespace std;


void pmc_heu::branch(vector<Vertex>& P, int sz,
        int& mc, vector<int>& C, vector<short>& ind) {

    if (P.size() > 0) {

        int u = P.back().get_id();
        P.pop_back();

        for (long long j = (*V)[u]; j < (*V)[u + 1]; j++)  ind[(*E)[j]] = 1;

        vector <Vertex> R;
        R.reserve(P.size());
        for (int i = 0; i < P.size(); i++)
            if (ind[P[i].get_id()])
                if ((*K)[P[i].get_id()] > mc)
                    R.push_back(P[i]);

        for (long long j = (*V)[u]; j < (*V)[u + 1]; j++)  ind[(*E)[j]] = 0;

        int mc_prev = mc;
        branch(R, sz + 1, mc, C, ind);

        if (mc > mc_prev)  C.push_back(u);

        R.clear();  P.clear();
    }
    else if (sz > mc)
        mc = sz;
    return;
}

int pmc_heu::search_bounds(pmc_graph& G,
        vector<int>& C_max) {

    V = G.get_vertices();
    E = G.get_edges();
    degree = G.get_degree();
    vector <int> C, X;
    C.reserve(ub);
    C_max.reserve(ub);
    vector<Vertex> P, T;
    P.reserve(G.get_max_degree()+1);
    T.reserve(G.get_max_degree()+1);
    vector<short> ind(G.num_vertices(),0);

    bool found_ub = false;
    int mc, mc_prev, mc_cur, i, v, k, lb_idx = 0;
    mc = lb > 0 ? lb - 1 : 0;

    #pragma omp parallel for schedule(dynamic) \
        shared(G, X, mc, C_max, lb_idx) private(i, v, P, mc_prev, mc_cur, C, k) firstprivate(ind) \
        num_threads(num_threads)
    for (i = G.num_vertices()-1; i >= 0; --i) {
        if (found_ub) continue;

        v = (*order)[i];
        mc_prev = mc_cur = mc;

        if ((*K)[v] > mc) {
            for (long long j = (*V)[v]; j < (*V)[v + 1]; j++)
                if ((*K)[(*E)[j]] > mc)
                    P.push_back( Vertex((*E)[j], compute_heuristic((*E)[j])) );


            if (P.size() > mc_cur) {
                std::sort(P.begin(), P.end(), incr_heur);
                branch(P, 1 , mc_cur, C, ind);

                if (mc_cur > mc_prev) {
                    if (mc < mc_cur) {
                        #pragma omp critical
                        if (mc < mc_cur) {
                            mc = mc_cur;
                            C.push_back(v);
                            C_max = C;
                            if (mc >= ub) found_ub = true;
                            print_info(C_max);
                        }
                    }
                }
            }
            C = X; P = T;
        }
    }
    DEBUG_PRINTF("[pmc heuristic]\t mc = %i\n", mc);
    return mc;
}


int pmc_heu::compute_heuristic(int v) {
    if (strat == "kcore_deg") 	return (*K)[v] * (*degree)[v];
    else if (strat == "deg")    return (*degree)[v];
    else if (strat == "kcore") 	return (*K)[v];
    else if (strat == "rand")  	return rand() % (*V).size();
    else if (strat == "var")    return (*K)[v] * ((int)(*degree)[v]/(*K)[v]);
    return v;
}


int pmc_heu::search_cores(pmc_graph& G, vector<int>& C_max, int lb) {

    vector <int> C, X;
    C.reserve(ub);
    C_max.reserve(ub);
    vector<Vertex> P, T;
    P.reserve(G.get_max_degree()+1);
    T.reserve(G.get_max_degree()+1);
    vector<short> ind(G.num_vertices(),0);

    int mc = lb, mc_prev, mc_cur, i;
    int lb_idx = 0, v = 0;
    for (i = G.num_vertices()-1; i >= 0; i--) {
        v = (*order)[i];
        if ((*K)[v] == lb)   lb_idx = i;
    }

    #pragma omp parallel for schedule(dynamic) \
        shared(G, X, mc, C_max) private(i, v, P, mc_prev, mc_cur, C) firstprivate(ind) num_threads(num_threads)
    for (i = lb_idx; i <= G.num_vertices()-1; i++) {

        v = (*order)[i];
        mc_prev = mc_cur = mc;

        if ((*K)[v] > mc_cur) {
            for (long long j = (*V)[v]; j < (*V)[v + 1]; j++)
                if ((*K)[(*E)[j]] > mc_cur)
                    P.push_back( Vertex((*E)[j], compute_heuristic((*E)[j])) );

            if (P.size() > mc_cur) {
                std::sort(P.begin(), P.end(), incr_heur);
                branch(P, 1 , mc_cur, C, ind);

                if (mc_cur > mc_prev) {
                    if (mc < mc_cur) {
                        #pragma omp critical
                        if (mc < mc_cur) {
                            mc = mc_cur;
                            C.push_back(v);
                            C_max = C;
                            print_info(C_max);
                        }
                    }
                }
            }
        }
        C = X; P = T;
    }
    C.clear();
    DEBUG_PRINTF("[search_cores]\t mc = %i\n", mc);
    return mc;
}


int pmc_heu::search(pmc_graph& G, vector<int>& C_max) {
    return search_bounds(G, C_max);
}


inline void pmc_heu::print_info(vector<int> C_max) {
    DEBUG_PRINTF("*** [pmc heuristic: thread %i", omp_get_thread_num() + 1);
    DEBUG_PRINTF("]   current max clique = %i", C_max.size());
    DEBUG_PRINTF(",  time = %i sec\n", get_time() - sec);
}

