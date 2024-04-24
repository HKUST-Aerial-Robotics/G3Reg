#include "back_end/mac3d/mac_reg.h"

namespace mac_reg{
    Eigen::Matrix4d MaximalCliqueReg::run(){

        Eigen::MatrixXf Graph = Graph_construction(correspondence, use_sc2);
        if (Graph.norm() == 0) {
            LOG(INFO) << "Graph is disconnected.";
            return Eigen::Matrix4d::Identity();
        }

        int total_num = correspondence.size();
        std::vector<int> degree(total_num, 0);
        std::vector<Vote_exp> pts_degree;
        for (int i = 0; i < total_num; i++) {
            Vote_exp t;
            t.true_num = 0;
            std::vector<int> corre_index;
            for (int j = 0; j < total_num; j++) {
                if (i != j && Graph(i, j)) {
                    degree[i]++;
                    corre_index.push_back(j);
                }
            }
            t.index = i;
            t.degree = degree[i];
            t.corre_index = corre_index;
            pts_degree.push_back(t);
        }

        //evaluate graph
        std::vector<Vote> cluster_factor;
        double sum_fenzi = 0;
        double sum_fenmu = 0;
        omp_set_num_threads(12);
        for (int i = 0; i < total_num; i++) {
            Vote t;
            double sum_i = 0;
            double wijk = 0;
            int index_size = pts_degree[i].corre_index.size();
#pragma omp parallel
            {
#pragma omp for
                for (int j = 0; j < index_size; j++) {
                    int a = pts_degree[i].corre_index[j];
                    for (int k = j + 1; k < index_size; k++) {
                        int b = pts_degree[i].corre_index[k];
                        if (Graph(a, b)) {
#pragma omp critical
                            wijk += pow(Graph(i, a) * Graph(i, b) * Graph(a, b), 1.0 / 3); //wij + wik
                        }
                    }
                }
            }

            if (degree[i] > 1) {
                double f1 = wijk;
                double f2 = degree[i] * (degree[i] - 1) * 0.5;
                sum_fenzi += f1;
                sum_fenmu += f2;
                double factor = f1 / f2;
                t.index = i;
                t.score = factor;
                cluster_factor.push_back(t);
            } else {
                t.index = i;
                t.score = 0;
                cluster_factor.push_back(t);
            }
        }
        double average_factor = 0;
        for (size_t i = 0; i < cluster_factor.size(); i++) {
            average_factor += cluster_factor[i].score;
        }
        average_factor /= cluster_factor.size();

        double total_factor = sum_fenzi / sum_fenmu;

        std::vector<Vote_exp> pts_degree_bac;
        std::vector<Vote> cluster_factor_bac;
        pts_degree_bac.assign(pts_degree.begin(), pts_degree.end());
        cluster_factor_bac.assign(cluster_factor.begin(), cluster_factor.end());

        sort(cluster_factor.begin(), cluster_factor.end(), compare_vote_score);
        sort(pts_degree.begin(), pts_degree.end(), compare_vote_degree);

        Eigen::VectorXd cluster_coefficients;
        cluster_coefficients.resize(cluster_factor.size());
        for (size_t i = 0; i < cluster_factor.size(); i++) {
            cluster_coefficients[i] = cluster_factor[i].score;
        }

        double OTSU = 0;
        if (cluster_factor[0].score != 0) {
            OTSU = OTSU_thresh(cluster_coefficients);
        }
        double cluster_threshold = std::min(OTSU, std::min(average_factor, total_factor));
        double weight_thresh = cluster_threshold;

        weight_thresh = 0;
        for (size_t i = 0; i < total_num; i++) {
            correspondence[i].score = cluster_factor_bac[i].score;
        }

        //GTM 筛选
        std::vector<int> Match_inlier;
        /*****************************************igraph**************************************************/
        igraph_t g;
        igraph_matrix_t g_mat;
        igraph_vector_t weights;
        igraph_vector_init(&weights, Graph.rows() * (Graph.cols() - 1) / 2);
        igraph_matrix_init(&g_mat, Graph.rows(), Graph.cols());

        if (cluster_threshold > 3 &&
            correspondence.size() > 50/*max(OTSU, total_factor) > 0.3*/) //reduce the graph size
        {
            double f = 10;
            while (1) {
                if (f * std::max(OTSU, total_factor) > cluster_factor[49].score) {
                    f -= 0.05;
                } else {
                    break;
                }
            }
            for (int i = 0; i < Graph.rows(); i++) {
                if (cluster_factor_bac[i].score > f * std::max(OTSU, total_factor)) {
                    for (int j = i + 1; j < Graph.cols(); j++) {
                        if (cluster_factor_bac[j].score > f * std::max(OTSU, total_factor)) {
                            MATRIX(g_mat, i, j) = Graph(i, j);
                        }
                    }
                }
            }
        } else {
            for (int i = 0; i < Graph.rows(); i++) {
                for (int j = i + 1; j < Graph.cols(); j++) {
                    if (Graph(i, j)) {
                        MATRIX(g_mat, i, j) = Graph(i, j);
                    }
                }
            }
        }

        igraph_set_attribute_table(&igraph_cattribute_table);
        igraph_weighted_adjacency(&g, &g_mat, IGRAPH_ADJ_UNDIRECTED, 0, 1);
        const char *att = "weight";
        EANV(&g, att, &weights);

        //find all maximal cliques
        igraph_vector_ptr_t cliques;
        igraph_vector_ptr_init(&cliques, 0);

        igraph_maximal_cliques(&g, &cliques, 3, 0); //3dlomatch 4 3dmatch; 3 Kitti  4
        int clique_num = igraph_vector_ptr_size(&cliques);
        if (clique_num == 0) {
            std::cerr << " NO CLIQUES! " << std::endl;
        }

        //clear useless data
        igraph_destroy(&g);
        igraph_matrix_destroy(&g_mat);
        igraph_vector_destroy(&weights);

        std::vector<int> remain;
        for (int i = 0; i < clique_num; i++) {
            remain.push_back(i);
        }
        node_cliques *N_C = new node_cliques[(int) total_num];
        int max_est_num = INT_MAX;
        find_largest_clique_of_node(Graph, &cliques, correspondence, N_C, remain, total_num, max_est_num);

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_corr_pts(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr des_corr_pts(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < correspondence.size(); i++) {
            src_corr_pts->push_back(correspondence[i].src);
            des_corr_pts->push_back(correspondence[i].des);
        }

        /******************************************registraion***************************************************/
        Eigen::Matrix4d best_est;
        double best_score = 0;
        double inlier_thresh = 0.6;
        std::vector<Corre_3DMatch> selected;
        std::vector<int> corre_index;
#pragma omp parallel for
        for (int i = 0; i < remain.size(); i++) {
            std::vector<Corre_3DMatch> Group;
            std::vector<int> selected_index;
            igraph_vector_t *v = (igraph_vector_t *) VECTOR(cliques)[remain[i]];
            int group_size = igraph_vector_size(v);
            for (int j = 0; j < group_size; j++) {
                Corre_3DMatch C = correspondence[VECTOR(*v)[j]];
                Group.push_back(C);
                selected_index.push_back(VECTOR(*v)[j]);
            }
            //igraph_vector_destroy(v);
            Eigen::Matrix4d est_trans;
            //evaluate cliques
            double score = evaluation_trans(Group, src_corr_pts, des_corr_pts, weight_thresh, est_trans,
                                            inlier_thresh);
            //GT未知
            if (score > 0) {
#pragma omp critical
                {
                    if (best_score < score) {
                        best_score = score;
                        best_est = est_trans;
                        selected = Group;
                        corre_index = selected_index;
                    }
                }
            }
            Group.clear();
            Group.shrink_to_fit();
            selected_index.clear();
            selected_index.shrink_to_fit();
        }
        //free memory
        igraph_vector_ptr_destroy(&cliques);
        post_refinement(src_corr_pts, des_corr_pts, best_est, best_score, inlier_thresh, 20, "MAE");

        correspondence.clear();
        correspondence.shrink_to_fit();
        degree.clear();
        degree.shrink_to_fit();
        pts_degree.clear();
        pts_degree.shrink_to_fit();
        pts_degree_bac.clear();
        pts_degree_bac.shrink_to_fit();
        cluster_factor.clear();
        cluster_factor.shrink_to_fit();
        cluster_factor_bac.clear();
        cluster_factor_bac.shrink_to_fit();
        delete[] N_C;
        remain.clear();
        remain.shrink_to_fit();
        selected.clear();
        selected.shrink_to_fit();
        corre_index.clear();
        corre_index.shrink_to_fit();
        src_corr_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
        des_corr_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
        return best_est;
    }

    Eigen::MatrixXf MaximalCliqueReg::Graph_construction(std::vector<Corre_3DMatch> &correspondence, bool sc2) {
        int size = correspondence.size();
        Eigen::MatrixXf cmp_score;
        cmp_score.resize(size, size);
        cmp_score.setZero();
        Corre_3DMatch c1, c2;
        float score, src_dis, des_dis, dis, alpha_dis;
        float thresh = 0.9; //fcgf 0.999 fpfh 0.9
        for (int i = 0; i < size; i++) {
            c1 = correspondence[i];
            for (int j = i + 1; j < size; j++) {
                c2 = correspondence[j];
                src_dis = Distance(c1.src, c2.src);
                des_dis = Distance(c1.des, c2.des);
                dis = abs(src_dis - des_dis);
                score = 1 - (dis * dis) / (0.6 * 0.6);
                //score = exp(-dis * dis);
                score = (score < thresh) ? 0 : score;//fcgf 0.9999 fpfh 0.9
                cmp_score(i, j) = score;
                cmp_score(j, i) = score;

            }
        }
        if (sc2) {
            //Eigen::setNbThreads(6);
            cmp_score = cmp_score.cwiseProduct(cmp_score * cmp_score);
        }
        return cmp_score;
    }

    void MaximalCliqueReg::post_refinement(pcl::PointCloud<pcl::PointXYZ>::Ptr &src_corr_pts, pcl::PointCloud<pcl::PointXYZ>::Ptr &des_corr_pts,
                                           Eigen::Matrix4d &initial, double &best_score, double inlier_thresh, int iterations,
                                           const std::string &metric) {
        int pointNum = src_corr_pts->points.size();
        double pre_score = best_score;
        for (int i = 0; i < iterations; i++) {
            double score = 0;
            Eigen::VectorXd weights, weight_pred;
            weights.resize(pointNum);
            weights.setZero();
            std::vector<int> pred_inlier_index;
            pcl::PointCloud<pcl::PointXYZ>::Ptr trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*src_corr_pts, *trans, initial);
            for (int j = 0; j < pointNum; j++) {
                double dist = Distance(trans->points[j], des_corr_pts->points[j]);
                double w = 1;
                if (dist < inlier_thresh) {
                    pred_inlier_index.push_back(j);
                    weights[j] = 1 / (1 + pow(dist / inlier_thresh, 2));
                    if (metric == "inlier") {
                        score += 1 * w;
                    } else if (metric == "MAE") {
                    } else if (metric == "MSE") {
                        score += pow((inlier_thresh - dist), 2) * w / pow(inlier_thresh, 2);
                    }
                }
            }
            if (score < pre_score) {
                break;
            } else {
                pre_score = score;
                //估计pred_inlier
                pcl::PointCloud<pcl::PointXYZ>::Ptr pred_src_pts(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr pred_des_pts(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*src_corr_pts, pred_inlier_index, *pred_src_pts);
                pcl::copyPointCloud(*des_corr_pts, pred_inlier_index, *pred_des_pts);
                weight_pred.resize(pred_inlier_index.size());
                for (int k = 0; k < pred_inlier_index.size(); k++) {
                    weight_pred[k] = weights[pred_inlier_index[k]];
                }
                //weighted_svd
                weight_SVD(pred_src_pts, pred_des_pts, weight_pred, 0, initial);
                pred_src_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
                pred_des_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
            }
            pred_inlier_index.clear();
            trans.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        best_score = pre_score;
    }

    double MaximalCliqueReg::Distance(pcl::PointXYZ &A, pcl::PointXYZ &B) {
        double distance = 0;
        double d_x = (double) A.x - (double) B.x;
        double d_y = (double) A.y - (double) B.y;
        double d_z = (double) A.z - (double) B.z;
        distance = sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
        return distance;
    }

    double MaximalCliqueReg::OTSU_thresh(Eigen::VectorXd values) {
        int i;
        int Quant_num = 100;
        double score_sum = 0.0;
        double fore_score_sum = 0.0;
        std::vector<int> score_Hist(Quant_num, 0);
        std::vector<double> score_sum_Hist(Quant_num, 0.0);
        double max_score_value, min_score_value;
        std::vector<double> all_scores;
        for (i = 0; i < values.size(); i++) {
            score_sum += values[i];
            all_scores.push_back(values[i]);
        }
        sort(all_scores.begin(), all_scores.end());
        max_score_value = all_scores[all_scores.size() - 1];
        min_score_value = all_scores[0];
        double Quant_step = (max_score_value - min_score_value) / Quant_num;
        for (i = 0; i < values.size(); i++) {
            int ID = values[i] / Quant_step;
            if (ID >= Quant_num) ID = Quant_num - 1;
            score_Hist[ID]++;
            score_sum_Hist[ID] += values[i];
        }
        double fmax = -1000;
        int n1 = 0, n2;
        double m1, m2, sb;
        double thresh = (max_score_value - min_score_value) / 2;//default value
        for (i = 0; i < Quant_num; i++) {
            double Thresh_temp = i * (max_score_value - min_score_value) / double(Quant_num);
            n1 += score_Hist[i];
            if (n1 == 0) continue;
            n2 = values.size() - n1;
            if (n2 == 0) break;
            fore_score_sum += score_sum_Hist[i];
            m1 = fore_score_sum / n1;
            m2 = (score_sum - fore_score_sum) / n2;
            sb = (double) n1 * (double) n2 * pow(m1 - m2, 2);
            if (sb > fmax) {
                fmax = sb;
                thresh = Thresh_temp;
            }
        }
        return thresh;
    }

    void
    MaximalCliqueReg::find_largest_clique_of_node(Eigen::MatrixXf &Graph, igraph_vector_ptr_t *cliques, std::vector<Corre_3DMatch> &correspondence,
                                                  node_cliques *result, std::vector<int> &remain, int num_node, int est_num) {
        int *vis = new int[igraph_vector_ptr_size(cliques)];
        memset(vis, 0, igraph_vector_ptr_size(cliques));
#pragma omp parallel for
        for (int i = 0; i < num_node; i++) {
            result[i].clique_index = -1;
            result[i].clique_size = 0;
            result[i].clique_weight = 0;
            result[i].clique_num = 0;
        }

        for (int i = 0; i < remain.size(); i++) {
            igraph_vector_t *v = (igraph_vector_t *) VECTOR(*cliques)[remain[i]];
            float weight = 0;
            int length = igraph_vector_size(v);
            for (int j = 0; j < length; j++) {
                int a = (int) VECTOR(*v)[j];
                for (int k = j + 1; k < length; k++) {
                    int b = (int) VECTOR(*v)[k];
                    weight += Graph(a, b);
                }
            }
            for (int j = 0; j < length; j++) {
                int k = (int) VECTOR(*v)[j];
                if (result[k].clique_weight < weight) {
                    result[k].clique_index = remain[i];
                    vis[remain[i]]++;
                    result[k].clique_size = length;
                    result[k].clique_weight = weight;
                }
            }
        }

#pragma omp parallel for
        for (int i = 0; i < remain.size(); i++) {
            if (vis[remain[i]] == 0) {
                igraph_vector_t *v = (igraph_vector_t *) VECTOR(*cliques)[remain[i]];
                igraph_vector_destroy(v);
            }
        }

        std::vector<int> after_delete;
        for (int i = 0; i < num_node; i++) {
            if (result[i].clique_index < 0) {
                continue;
            }
            if (vis[result[i].clique_index] > 0) {
                vis[result[i].clique_index] = 0;
                after_delete.push_back(result[i].clique_index);
            } else if (vis[result[i].clique_index] == 0) {
                result[i].clique_index = -1;
            }
        }
        remain.clear();
        remain = after_delete;

        //reduce the number of cliques
        if (remain.size() > est_num) {
            std::vector<int> after_decline;
            std::vector<Vote> clique_score;
            for (int i = 0; i < num_node; i++) {
                if (result[i].clique_index < 0) {
                    continue;
                }
                Vote t;
                t.index = result[i].clique_index;
                t.score = result[i].clique_weight;
                clique_score.push_back(t);
            }
            sort(clique_score.begin(), clique_score.end(), compare_vote_score);
            for (int i = 0; i < est_num; i++) {
                after_decline.push_back(clique_score[i].index);
            }
            remain.clear();
            remain = after_decline;
            clique_score.clear();
        }
        delete[] vis;
        return;
    }

    double
    MaximalCliqueReg::evaluation_trans(std::vector<Corre_3DMatch> &Match, pcl::PointCloud<pcl::PointXYZ>::Ptr &src_corr_pts,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &des_corr_pts, double weight_thresh, Eigen::Matrix4d &trans, double metric_thresh) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_pts(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr des_pts(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<double> weights;
        for (auto &i: Match) {
            if (i.score >= weight_thresh) {
                src_pts->push_back(i.src);
                des_pts->push_back(i.des);
                weights.push_back(i.score);
            }
        }
        if (weights.size() < 3) {
            return 0;
        }
        Eigen::VectorXd weight_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(weights.data(), weights.size());
        weights.clear();
        weights.shrink_to_fit();
        weight_vec /= weight_vec.maxCoeff();
        weight_vec.setOnes(); // 2023.2.23
        weight_SVD(src_pts, des_pts, weight_vec, 0, trans);
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_trans(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*src_corr_pts, *src_trans, trans);
        //Eigen::Matrix4f trans_f = trans.cast<float>();
        //Eigen::Matrix3f R = trans_f.topLeftCorner(3, 3);
        double score = 0.0;
        int inlier = 0;
        int corr_num = src_corr_pts->points.size();
        for (int i = 0; i < corr_num; i++) {
            double dist = Distance(src_trans->points[i], des_corr_pts->points[i]);
            double w = 1;
            if (dist < metric_thresh) {
                inlier++;
                score += (metric_thresh - dist) * w / metric_thresh;
            }
        }
        src_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
        des_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
        src_trans.reset(new pcl::PointCloud<pcl::PointXYZ>);
        return score;
    }

    void MaximalCliqueReg::weight_SVD(pcl::PointCloud<pcl::PointXYZ>::Ptr &src_pts, pcl::PointCloud<pcl::PointXYZ>::Ptr &des_pts, Eigen::VectorXd &weights, double weight_threshold,
                                      Eigen::Matrix4d &trans_Mat) {
        for (size_t i = 0; i < weights.size(); i++) {
            weights(i) = (weights(i) < weight_threshold) ? 0 : weights(i);
        }

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> weight;
        Eigen::VectorXd ones = weights;
        ones.setOnes();
        weight = (weights * ones.transpose());
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Identity = weight;

        Identity.setIdentity();
        weight = (weights * ones.transpose()).cwiseProduct(Identity);
        pcl::ConstCloudIterator<pcl::PointXYZ> src_it(*src_pts);
        pcl::ConstCloudIterator<pcl::PointXYZ> des_it(*des_pts);

        src_it.reset();
        des_it.reset();
        Eigen::Matrix<double, 4, 1> centroid_src, centroid_des;
        pcl::compute3DCentroid(src_it, centroid_src);
        pcl::compute3DCentroid(des_it, centroid_des);

        src_it.reset();
        des_it.reset();
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> src_demean, des_demean;
        pcl::demeanPointCloud(src_it, centroid_src, src_demean);
        pcl::demeanPointCloud(des_it, centroid_des, des_demean);

        Eigen::Matrix<double, 3, 3> H = (src_demean * weight * des_demean.transpose()).topLeftCorner(3, 3);

        // Compute the Singular Value Decomposition
        Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<double, 3, 3> u = svd.matrixU();
        Eigen::Matrix<double, 3, 3> v = svd.matrixV();

        // Compute R = V * U'
        if (u.determinant() * v.determinant() < 0) {
            for (int x = 0; x < 3; ++x)
                v(x, 2) *= -1;
        }

        Eigen::Matrix<double, 3, 3> R = v * u.transpose();

        // Return the correct transformation
        Eigen::Matrix<double, 4, 4> Trans;
        Trans.setIdentity();
        Trans.topLeftCorner(3, 3) = R;
        const Eigen::Matrix<double, 3, 1> Rc(R * centroid_src.head(3));
        Trans.block(0, 3, 3, 1) = centroid_des.head(3) - Rc;
        trans_Mat = Trans;
    }

    void solve(const std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, const std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes,
               const clique_solver::Association& associations, FRGresult& result){

        robot_utils::TicToc tt;
        std::vector<MaximalCliqueReg::Corre_3DMatch> correspondence;
        for (int i=0;i<associations.rows();i++){
            MaximalCliqueReg::Corre_3DMatch t;
            t.src = pcl::PointXYZ(src_nodes[associations(i,0)]->centroid(0), src_nodes[associations(i,0)]->centroid(1), src_nodes[associations(i,0)]->centroid(2));
            t.des = pcl::PointXYZ(tgt_nodes[associations(i,1)]->centroid(0), tgt_nodes[associations(i,1)]->centroid(1), tgt_nodes[associations(i,1)]->centroid(2));
            t.score = 0;
            correspondence.push_back(t);
        }
        MaximalCliqueReg mcr(true);
        mcr.setCorrespondence(correspondence);
        Eigen::Matrix4d best_tf = mcr.run();

        result.tf_solver_time = tt.toc();
        result.tf = best_tf;
        result.candidates = { best_tf };
    }

}