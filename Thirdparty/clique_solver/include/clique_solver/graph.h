/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#pragma once

#include <unordered_set>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "macros.h"
#include <fstream>
#include <boost/shared_ptr.hpp>

namespace clique_solver {

    using Association = Eigen::MatrixX2i;
    using Affinity = Eigen::MatrixXd;
    using Constraint = Eigen::MatrixXd;
    using SpMat = Eigen::SparseMatrix<double>;
    using SpAffinity = SpMat;
    using SpConstraint = SpMat;

    /**
     * A simple undirected graph class
     *
     * This graph assumes that vertices are numbered. In addition, the vertices numbers have to be
     * consecutive starting from 0.
     *
     * For example, if the graph have 3 vertices, they have to be named 0, 1, and 2.
     */
    class Graph {

    public:
        using Ptr = boost::shared_ptr<Graph>;

        Graph() : num_edges_(0) {
            use_adj_matrix_ = false;
        };

        /**
         * Constructor that takes in an adjacency list. Notice that for an edge connecting two arbitrary
         * vertices v1 & v2, we assume that v2 exists in v1's list, and v1 also exists in v2's list. This
         * condition is not enforced. If violated, removeEdge() function might exhibit undefined
         * behaviors.
         * @param [in] adj_list an map representing an adjacency list
         */
        explicit Graph(const std::map<int, std::vector<int>> &adj_list) {
            adj_list_.resize(adj_list.size());
            num_edges_ = 0;
            for (const auto &e_list: adj_list) {
                const auto &v = e_list.first;
                adj_list_[e_list.first] = e_list.second;
                num_edges_ += e_list.second.size();
            }
            num_edges_ /= 2;
        };

        /**
         * Add a vertex with no edges.
         * @param [in] id the id of vertex to be added
         */
        void addVertex(const int &id) {
            if (id < adj_list_.size()) {
                TEASER_DEBUG_ERROR_MSG("Vertex already exists.");
            } else {
                adj_list_.resize(id + 1);
            }
        }

        /**
         * Populate the graph with the provided number of vertices without any edges.
         * @param num_vertices
         */
        void populateVertices(const int &num_vertices) {
            adj_list_.resize(num_vertices);
            for (int i = 0; i < num_vertices; ++i) {
                adj_list_[i].reserve(num_vertices);
            }
            if (use_adj_matrix_) {
                M_ = Eigen::MatrixXd::Zero(num_vertices, num_vertices);
            }
        }

        /**
         * Return true if said edge exists
         * @param [in] vertex_1
         * @param [in] vertex_2
         */
        bool hasEdge(const int &vertex_1, const int &vertex_2) {
            if (vertex_1 >= adj_list_.size() || vertex_2 >= adj_list_.size()) {
                return false;
            }

            if (use_adj_matrix_) {
                if (vertex_1 <= vertex_2) {
                    return M_(vertex_1, vertex_2) != 0;
                } else {
                    return M_(vertex_2, vertex_1) != 0;
                }
            }

            auto &connected_vs = adj_list_[vertex_1];
            bool exists =
                    std::find(connected_vs.begin(), connected_vs.end(), vertex_2) != connected_vs.end();
            return exists;
        }

        /**
         * Return true if the vertex exists.
         * @param vertex
         * @return
         */
        bool hasVertex(const int &vertex) { return vertex < adj_list_.size(); }

        /**
         * Add an edge between two vertices
         * @param [in] vertex_1 one vertex of the edge
         * @param [in] vertex_2 another vertex of the edge
         */
        void addEdge(const int &vertex_1, const int &vertex_2) {
            if (hasEdge(vertex_1, vertex_2)) {
                TEASER_DEBUG_ERROR_MSG("Edge exists.");
                return;
            }
            adj_list_[vertex_1].push_back(vertex_2);
            adj_list_[vertex_2].push_back(vertex_1);
            num_edges_++;
            if (use_adj_matrix_){
                if (vertex_1 <= vertex_2) {
                    M_(vertex_1, vertex_2) = 1.0;
                } else{
                    M_(vertex_2, vertex_1) = 1.0;
                }
            }
        }

        /**
         * Add an edge between two vertices
         * @param [in] vertex_1 one vertex of the edge
         * @param [in] vertex_2 another vertex of the edge
         */
        void addEdge(const int &vertex_1, const int &vertex_2, const double &weight) {
            if (hasEdge(vertex_1, vertex_2)) {
                TEASER_DEBUG_ERROR_MSG("Edge exists.");
                return;
            }
            adj_list_[vertex_1].push_back(vertex_2);
            adj_list_[vertex_2].push_back(vertex_1);
            num_edges_++;
            if (use_adj_matrix_) {
                if (vertex_1 <= vertex_2) {
                    M_(vertex_1, vertex_2) = weight;
                } else{
                    M_(vertex_2, vertex_1) = weight;
                }
            }
        }

		// Method to prune the graph safely
		void pruneGraph(int degree) {
			std::vector<int> verticesToRemove;
		
			// Step 1: Find all vertices that need to be removed
			for (size_t i = 0; i < adj_list_.size(); ++i) {
				if (adj_list_[i].size() < static_cast<size_t>(degree)) {
					verticesToRemove.push_back(i);
				}
			}
		
			// Step 2: Remove the vertices and associated edges
			// Iterate in reverse to avoid shifting indices of yet-to-be-removed vertices
			for (auto it = verticesToRemove.rbegin(); it != verticesToRemove.rend(); ++it) {
				removeVertex(*it);
			}
		}
	
		// Method to safely remove a vertex and its associated edges
		void removeVertex(int vertex) {
			// Remove all edges associated with the vertex
			for (auto& edges : adj_list_) {
				edges.erase(std::remove(edges.begin(), edges.end(), vertex), edges.end());
			}
		
			// Since we're using a vector, we can't just leave an empty slot,
			// we need to remove the vertex and update indices.
			// This requires updating all higher-indexed vertices in all edges.
			if (vertex < adj_list_.size()) {
				// Remove the vertex's adjacency list
				adj_list_.erase(adj_list_.begin() + vertex);
			
				// Update the indices in the adjacency lists
				for (auto& edges : adj_list_) {
					for (auto& e : edges) {
						if (e > vertex) {
							--e; // Decrement the stored index
						}
					}
				}
			}
		}

        void printStatistics() {
            std::cout << "Number of vertices: " << numVertices() << std::endl;
            std::cout << "Number of edges: " << numEdges() << std::endl;

            std::map<int, int> degree_count;
            for (int i = 0; i < adj_list_.size(); ++i) {
                if (degree_count.find(adj_list_[i].size()) == degree_count.end()) {
                    degree_count[adj_list_[i].size()] = 1;
                } else {
                    degree_count[adj_list_[i].size()]++;
                }
            }
            std::cout << "Degree distribution: ";
            int total_degree = 0;
            for (const auto &dc: degree_count) {
                std::cout << dc.first << ": " << dc.second << ", ";
                total_degree += dc.first * dc.second;
            }
            std::cout << std::endl;
            std::cout << "Average degree: " << total_degree / adj_list_.size() << ", total degree: " << total_degree << std::endl;
        }

        void setType(const bool &use_adj_matrix) { use_adj_matrix_ = use_adj_matrix; }

        void initAffinityMatrix(){
            if (!use_adj_matrix_) return;
            // make the diagonal elements of M_ to be 0
            for (int i = 0; i < M_.rows(); ++i) {
                M_(i, i) = 0;
            }
            affinity_ = M_.sparseView();
            constraint_ = affinity_;
            constraint_.coeffs() = 1;
        }

        SpAffinity affinity() const { return affinity_; }

        SpConstraint constraint() const { return constraint_; }

        void setAffinity(const SpAffinity &affinity) { affinity_ = affinity; }

        void setConstraint(const SpConstraint &constraint) { constraint_ = constraint; }

        /**
         * Get the number of vertices
         * @return total number of vertices
         */
        // [[nodiscard]] tells the compiler if return value of the function is not used
        [[nodiscard]] int numVertices() const { return adj_list_.size(); }

        /**
         * Get the number of edges
         * @return total number of edges
         */
        [[nodiscard]] int numEdges() const { return num_edges_; }

        /**
         * Get edges originated from a specific vertex
         * @param [in] id
         * @return an unordered set of edges
         */
        [[nodiscard]] const std::vector<int> &getEdges(int id) const { return adj_list_[id]; }

        /**
         * Get all vertices
         * @return a vector of all vertices
         */
        [[nodiscard]] std::vector<int> getVertices() const {
            std::vector<int> v;
            for (int i = 0; i < adj_list_.size(); ++i) {
                v.push_back(i);
            }
            return v;
        }

        [[nodiscard]] Eigen::MatrixXi getAdjMatrix() const {
            const int num_v = numVertices();
            Eigen::MatrixXi adj_matrix(num_v, num_v);
            for (size_t i = 0; i < num_v; ++i) {
                const auto &c_edges = getEdges(i);
                for (size_t j = 0; j < num_v; ++j) {
                    if (std::find(c_edges.begin(), c_edges.end(), j) != c_edges.end()) {
                        adj_matrix(i, j) = 1;
                    } else {
                        adj_matrix(i, j) = 0;
                    }
                }
            }
            return adj_matrix;
        }

        [[nodiscard]] std::vector<std::vector<int>> getAdjList() const { return adj_list_; }

        /**
         * Preallocate spaces for vertices
         * @param num_vertices
         */
        void reserve(const int &num_vertices) { adj_list_.reserve(num_vertices); }

        /**
         * Clear the contents of the graph
         */
        void clear() {
            adj_list_.clear();
            num_edges_ = 0;
        }

        /**
         * Reserve space for complete graph. A complete undirected graph should have N*(N-1)/2 edges
         * @param num_vertices
         */
        void reserveForCompleteGraph(const int &num_vertices) {
            adj_list_.reserve(num_vertices);
            for (int i = 0; i < num_vertices - 1; ++i) {
                std::vector<int> c_edges;
                c_edges.reserve(num_vertices - 1);
                adj_list_.push_back(c_edges);
            }
            adj_list_.emplace_back(std::initializer_list<int>{});
        }

        void saveToFile(const std::string& file_path) {
            std::ofstream out_file(file_path);
            if (!out_file.is_open()) {
                TEASER_DEBUG_ERROR_MSG("Cannot open file " + file_path);
                return;
            }
            out_file << numVertices() << " " << numEdges() * 2 << std::endl;
            for (int i = 0; i < numVertices(); ++i) {
                const auto& c_edges = getEdges(i);
                for (const auto& e : c_edges) {
                    out_file << i << " " << e << std::endl;
                }
            }
            out_file.close();
        }

        void loadFromFile(const std::string& file_path) {
            std::ifstream in_file(file_path);
            if (!in_file.is_open()) {
                TEASER_DEBUG_ERROR_MSG("Cannot open file " + file_path);
                return;
            }
            clear();
            int num_v, num_e;
            in_file >> num_v >> num_e;
            populateVertices(num_v);
            for (int i = 0; i < num_e; ++i) {
                int v1, v2;
                in_file >> v1 >> v2;
                addEdge(v1, v2);
            }
            in_file.close();
        }

    private:
        std::vector<std::vector<int>> adj_list_;
        size_t num_edges_;
        SpAffinity affinity_;
        SpConstraint constraint_;
        Eigen::MatrixXd M_;
        bool use_adj_matrix_ = false;
    };

} // namespace clique_solver
