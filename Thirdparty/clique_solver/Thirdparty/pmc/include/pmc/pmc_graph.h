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

#ifndef PMC_GRAPH_H_
#define PMC_GRAPH_H_

#include <float.h>
#include <cstddef>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <limits>
#include "math.h"
#include "pmc_headers.h"
#include "pmc_utils.h"
#include "pmc_vertex.h"


namespace pmc {
    class pmc_graph {
        private:
            // helper functions
            void read_mtx(const string& filename);
            void read_edges(const string& filename);
            void read_metis(const string& filename);

        public:
            vector<int> edges;
            vector<long long> vertices;
            vector<int> degree;
            int min_degree;
            int max_degree;
            double avg_degree;
            bool is_gstats;
            string fn;
            vector<vector<bool>> adj;

            // constructor
            pmc_graph(const string& filename);
            pmc_graph(bool graph_stats, const string& filename);
            pmc_graph(const string& filename, bool make_adj);
            pmc_graph(vector<long long> vs, vector<int> es) {
                edges = std::move(es);
                vertices = std::move(vs);
                vertex_degrees();
            }
            pmc_graph(long long nedges, const int *ei, const int *ej, int offset);
            pmc_graph(map<int,vector<int> > v_map);
                
            // destructor
            ~pmc_graph();

            void read_graph(const string& filename);
            void create_adj();
            void reduce_graph(int* &pruned);
            void reduce_graph(
                    vector<long long>& vs,
                    vector<int>& es,
                    int* &pruned,
                    int id,
                    int& mc);

            int num_vertices() { return vertices.size() - 1; }
            int num_edges() { return edges.size()/2; }
            vector <long long>* get_vertices(){ return &vertices; }
            vector<int>* get_edges(){ return &edges; }
            vector<int>* get_degree(){ return &degree; }
            vector<int> get_edges_array() { return edges; }
            vector<long long> get_vertices_array() { return vertices; };
            vector<long long> e_v, e_u, eid;

            int vertex_degree(int v) { return vertices[v] - vertices[v+1]; }
            long long first_neigh(int v) { return vertices[v]; }
            long long last_neigh(int v) { return vertices[v+1]; }

            void sum_vertex_degrees();
            void vertex_degrees();
            void update_degrees();
            void update_degrees(bool flag);
            void update_degrees(int* &pruned, int& mc);
            double density() { return (double)num_edges() / (num_vertices() * (num_vertices() - 1.0) / 2.0); }
            int get_max_degree() { return max_degree; }
            int get_min_degree() { return min_degree; }
            double get_avg_degree() { return avg_degree; }

            void initialize();
            string get_file_extension(const string& filename);
            void basic_stats(double sec);
            void bound_stats(int alg, int lb, pmc_graph& G);

            // vertex sorter
            void compute_ordering(vector<int>& bound, vector<int>& order);
            void compute_ordering(string degree, vector<int>& order);
            // edge sorters
            void degree_bucket_sort();
            void degree_bucket_sort(bool desc);

            int max_core;
            vector<int> kcore;
            vector<int> kcore_order;
            vector<int>* get_kcores() { return &kcore; }
            vector<int>* get_kcore_ordering() { return &kcore_order; }
            int get_max_core() { return max_core; }
            void update_kcores(int* &pruned);

            void compute_cores();
            void induced_cores_ordering(
                    vector<long long>& V,
                    vector<int>& E,
                    int* &pruned);

            // clique utils
            int initial_pruning(pmc_graph& G, int* &pruned, int lb);
            int initial_pruning(pmc_graph& G, int* &pruned, int lb, vector<vector<bool>> &adj);
            void order_vertices(vector<Vertex> &V, pmc_graph &G,
                    int &lb_idx, int &lb, string vertex_ordering, bool decr_order);

            void print_info(vector<int> &C_max, double &sec);
            void print_break();
            bool time_left(vector<int> &C_max, double sec,
                    double time_limit, bool &time_expired_msg);
            void graph_stats(pmc_graph& G, int& mc, int id, double &sec);

            void reduce_graph(
                    vector<long long>& vs,
                    vector<int>& es,
                    int* &pruned,
                    pmc_graph& G,
                    int id,
                    int& mc);

            bool clique_test(pmc_graph& G, vector<int> C);
    };

}
#endif
