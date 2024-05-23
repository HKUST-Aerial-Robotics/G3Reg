#ifndef MAC_MAXIMALCLIQUEREG_H
#define MAC_MAXIMALCLIQUEREG_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <igraph/igraph.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include "front_end/gem/ellipsoid.h"

namespace mac_reg {

    class MaximalCliqueReg {

    public:
        bool use_sc2;

        typedef struct {
            int src_index;
            int des_index;
            pcl::PointXYZ src;
            pcl::PointXYZ des;
            Eigen::Vector3f src_norm;
            Eigen::Vector3f des_norm;
            Eigen::Matrix3f covariance_src, covariance_des;                    //创建3×3协方差矩阵存储对象
            Eigen::Vector4f centeroid_src, centeroid_des;                    //创建用于计算协方差矩阵的点云质心对象
            double score;
            int inlier_weight;
        } Corre_3DMatch;

        typedef struct {
            int index;
            int degree;
            double score;
            std::vector<int> corre_index;
            int true_num;
        } Vote_exp;

        typedef struct {
            int clique_index;
            int clique_size;
            float clique_weight;
            int clique_num;
        } node_cliques;

        typedef struct {
            int index;
            double score;
        } Vote;

        std::vector<Corre_3DMatch> correspondence;

    public:
        MaximalCliqueReg(bool use_sc2) : use_sc2(use_sc2) {}

        MaximalCliqueReg(const std::string &corr_path, bool use_sc2) : use_sc2(use_sc2) {

            FILE *corr;
            corr = fopen(corr_path.c_str(), "r");
            if (corr == NULL) {
                printf("Correspondence File can't open!\n");
            }

            while (!feof(corr)) {
                Corre_3DMatch t;
                pcl::PointXYZ src, des;
                fscanf(corr, "%f %f %f %f %f %f\n", &src.x, &src.y, &src.z, &des.x, &des.y, &des.z);
                t.src = src;
                t.des = des;
                t.score = 0;
                correspondence.push_back(t);
            }
            fclose(corr);
        }

        void setCorrespondence(std::vector<Corre_3DMatch> &correspondence) {
            this->correspondence = std::move(correspondence);
        }

        Eigen::Matrix4d run();

        Eigen::MatrixXf Graph_construction(std::vector<Corre_3DMatch> &correspondence, bool sc2);

        void post_refinement(pcl::PointCloud<pcl::PointXYZ>::Ptr &src_corr_pts,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &des_corr_pts,
                             Eigen::Matrix4d &initial, double &best_score, double inlier_thresh, int iterations,
                             const std::string &metric);

        double Distance(pcl::PointXYZ &A, pcl::PointXYZ &B);

        static bool compare_vote_score(const Vote &v1, const Vote &v2) {
            return v1.score > v2.score;
        }

        static bool compare_vote_degree(const Vote_exp &v1, const Vote_exp &v2) {
            return v1.degree > v2.degree;
        }

        double OTSU_thresh(Eigen::VectorXd values);

        void
        find_largest_clique_of_node(Eigen::MatrixXf &Graph, igraph_vector_ptr_t *cliques,
                                    std::vector<Corre_3DMatch> &correspondence,
                                    node_cliques *result, std::vector<int> &remain, int num_node, int est_num);

        double
        evaluation_trans(std::vector<Corre_3DMatch> &Match, pcl::PointCloud<pcl::PointXYZ>::Ptr &src_corr_pts,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &des_corr_pts, double weight_thresh,
                         Eigen::Matrix4d &trans, double metric_thresh);

        void weight_SVD(pcl::PointCloud<pcl::PointXYZ>::Ptr &src_pts, pcl::PointCloud<pcl::PointXYZ>::Ptr &des_pts,
                        Eigen::VectorXd &weights, double weight_threshold,
                        Eigen::Matrix4d &trans_Mat);
    };

    void solve(const std::vector<clique_solver::GraphVertex::Ptr> &src_nodes,
               const std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes,
               const clique_solver::Association &associations, FRGresult &result);
}


#endif //MAC_MAXIMALCLIQUEREG_H
