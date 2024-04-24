/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "front_end/graph_vertex.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

namespace clique_solver{

    GraphVertex::Ptr EllipseVertex::operator-(const GraphVertex& other) const{
        const Eigen::Vector3d& diff = centroid - other.centroid;
        const Eigen::Matrix3d& cov = covariance + other.covariance;
        VertexInfo vertex_info = vertex_info_;
        vertex_info.prior_bound = vertex_info_.prior_bound + other.vertex_info_.prior_bound;
        GraphVertex::Ptr new_vertex = GraphVertex::Ptr(new EllipseVertex(diff, cov, vertex_info));
        return new_vertex;
    }

    double EllipseVertex::upper_rho(const Eigen::Matrix3d& cov){
        // Perron–Frobenius theorem
        double upper_bound1 = cov.cwiseAbs().rowwise().sum().maxCoeff();
        // https://www.math.uwaterloo.ca/~hwolkowi/henry/reports/bndseigs80.pdf
        double m = cov.trace() / 3;
        double s2 = (cov * cov).trace() / 3 - m * m;
        double upper_bound2 = m + sqrt(2 * s2);
        return std::min({upper_bound1, upper_bound2});
    }

    Eigen::VectorXd EllipseVertex::consistent(const GraphVertex &other) {
        const double& v1i_dist = this->norm();
        const double& v2i_dist = other.norm();
        if (v1i_dist < 1e-6 || v2i_dist < 1e-6)
            return Eigen::VectorXd::Zero(num_graphs());

//        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es1(covariance);
//        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es2(other.covariance);
        //const double spec_radius1 = es1.eigenvalues().maxCoeff();
        //const double spec_radius2 = es2.eigenvalues().maxCoeff();
        const double spec_radius1 = std::min({upper_rho(covariance), vertex_info_.prior_bound});
        const double spec_radius2 = std::min({upper_rho(other.covariance), other.vertex_info_.prior_bound});

        // consistency score
        const double c = std::abs(v1i_dist - v2i_dist);
        Eigen::VectorXd prob = Eigen::VectorXd::Zero(num_graphs());
        // noise bound vector is in ascending order
        for (int level = 0; level < num_graphs(); ++level) {

            const double& chi_square = vertex_info_.noise_bound_vec[level];
            const double& upper_bound1 = sqrt(spec_radius1 * chi_square);
            const double& upper_bound2 = sqrt(spec_radius2 * chi_square);
            const double& beta = upper_bound1 + upper_bound2;

            if (c < beta) {
                prob(level) = 0.99;
                for (int j = level + 1; j < num_graphs(); ++j) {
                    prob(j) = prob(level);
                }
                break;
            } else {
                prob(level) = 0.0;
            }
        }
        return prob;
    }

    GraphVertex::Ptr GaussianVertex::operator-(const GraphVertex& other) const{
        const Eigen::Matrix3d& sum_covariance = covariance + other.covariance;
        const Eigen::Vector3d& diff = centroid - other.centroid;
        return GraphVertex::Ptr(new GaussianVertex(diff, sum_covariance, vertex_info_));
    }

    double GaussianVertex::wasserstein_distance(const GraphVertex& other){
        Eigen::Matrix3d temp = covariance + other.covariance - 2*(covariance.sqrt()*other.covariance*covariance.sqrt()).sqrt();
        return temp.trace();
    }

    Eigen::VectorXd GaussianVertex::consistent(const GraphVertex &other) {
        const double& v1i_dist = this->norm();
        const double& v2i_dist = other.norm();
        if (v1i_dist < 1e-6 || v2i_dist < 1e-6)
            return Eigen::VectorXd::Zero(num_graphs());

        const double& meas = std::abs(v1i_dist - v2i_dist);
        double waterstein_dist = this->wasserstein_distance(other);

        Eigen::VectorXd prob = Eigen::VectorXd::Zero(num_graphs());
//        const double& sigma2 = vertex_info_.sigma * vertex_info_.sigma;
        // noise bound vector is in ascending order
        for (int level = 0; level < num_graphs(); ++level) {
            const double& beta = 2 * vertex_info_.noise_bound_vec[level];
            if (meas < beta && waterstein_dist < 5.0) {
//                prob(level) = std::exp(-0.5 * (meas * meas) / sigma2);
                prob(level) = 0.99;
                for (int j = level + 1; j < num_graphs(); ++j) {
                    prob(j) = prob(level);
                }
                break;
            } else {
                prob(level) = 0.0;
            }
        }
        return prob;
    }

    GraphVertex::Ptr PointVertex::operator-(const GraphVertex& other) const{
        const Eigen::Vector3d& diff = centroid - other.centroid;
        return GraphVertex::Ptr(new PointVertex(diff, vertex_info_));
    }

    Eigen::VectorXd PointVertex::consistent(const GraphVertex &other) {
        const double& v1i_dist = this->norm();
        const double& v2i_dist = other.norm();
        if (v1i_dist < 1e-6 || v2i_dist < 1e-6)
            return Eigen::VectorXd::Zero(num_graphs());

        const double& meas = std::abs(v1i_dist - v2i_dist);

        Eigen::VectorXd prob = Eigen::VectorXd::Zero(num_graphs());
//        const double& sigma2 = vertex_info_.sigma * vertex_info_.sigma;
        // noise bound vector is in ascending order
        for (int level = 0; level < num_graphs(); ++level) {
            const double& beta = 2 * vertex_info_.noise_bound_vec[level];
            if (meas < beta) {
//                prob(level) = std::exp(-0.5 * (meas * meas) / sigma2);
                prob(level) = 0.99;
                for (int j = level + 1; j < num_graphs(); ++j) {
                    prob(j) = prob(level);
                }
                break;
            } else {
                prob(level) = 0.0;
            }
        }
        return prob;
    }

    GraphVertex::Ptr PointRatioVertex::operator-(const GraphVertex& other) const{
        const Eigen::Vector3d& diff = centroid - other.centroid;
        return GraphVertex::Ptr(new PointRatioVertex(diff, vertex_info_));
    }

    Eigen::VectorXd PointRatioVertex::consistent(const GraphVertex &other) {
        const double& v1i_dist = this->norm();
        const double& v2i_dist = other.norm();
        if (v1i_dist < 1e-6 || v2i_dist < 1e-6)
            return Eigen::VectorXd::Zero(num_graphs());

        const double& s1 = abs(v1i_dist / v2i_dist - 1);
        const double& s2 = abs(v2i_dist / v1i_dist - 1);

        Eigen::VectorXd prob = Eigen::VectorXd::Zero(num_graphs());
//        const double& sigma2 = vertex_info_.sigma * vertex_info_.sigma;
        // noise bound vector is in ascending order
        for (int level = 0; level < num_graphs(); ++level) {
            const double& beta = 2 * vertex_info_.noise_bound_vec[level];
            if (s1 < beta && s2 < beta) {
                const double& s = std::min(s1, s2);
//                prob(level) = std::exp(-0.5 * (s * s) / sigma2);
                prob(level) = 0.99;
                for (int j = level + 1; j < num_graphs(); ++j) {
                    prob(j) = prob(level);
                }
                break;
            } else {
                prob(level) = 0.0;
            }
        }
        return prob;
    }
	
	clique_solver::GraphVertex::Ptr create_vertex(const Eigen::Vector3d& center, const VertexInfo& vertex_info, const Eigen::Matrix3d& cov,
												  const double& prior_bound){
		clique_solver::GraphVertex::Ptr vertex;
		clique_solver::VertexInfo vertexInfo = vertex_info;
		if (vertexInfo.type == clique_solver::VertexType::GAUSSIAN){
			vertex = clique_solver::GraphVertex::Ptr(new clique_solver::GaussianVertex(center, cov, vertexInfo));
		} else if (vertexInfo.type == clique_solver::VertexType::POINT){
			vertex = clique_solver::GraphVertex::Ptr(new clique_solver::PointVertex(center, vertexInfo));
		} else if (vertexInfo.type == clique_solver::VertexType::POINT_RATIO){
			vertex = clique_solver::GraphVertex::Ptr(new clique_solver::PointRatioVertex(center, vertexInfo));
		} else if (vertexInfo.type == clique_solver::VertexType::ELLIPSE){
			vertexInfo.prior_bound = prior_bound;
			vertex = clique_solver::GraphVertex::Ptr(new clique_solver::EllipseVertex(center, cov, vertexInfo));
		} else {
			std::cerr << "Unknown vertex type" << std::endl;
		}
		return vertex;
	}
	
    std::tuple<size_t, size_t> k2ij(size_t k, size_t n) {
        k += 1;

        const size_t l = n * (n - 1) / 2 - k;
        const size_t o = std::floor((std::sqrt(1 + 8 * l) - 1) / 2.);
        const size_t p = l - o * (o + 1) / 2;
        const size_t i = n - (o + 1);
        const size_t j = n - p;
        return {i - 1, j - 1};
    }

    std::tuple<size_t, size_t, size_t> k2ijl(size_t k, size_t N, const std::vector<size_t>& indices) {
        // use binary search to find i
        int left = 0, right = N - 2;
        while (left < right)
        {
            int mid = (left + right) / 2;
            if (k < indices[mid])
            {
                right = mid;
            }
            else
            {
                left = mid + 1;
            }
        }
        int i = left, j = 0, l = 0;
        std::tie(j, l) = k2ij(k - (i == 0 ? 0 : indices[i - 1]), N - i - 1);
        j = j + i + 1;
        l = l + i + 1;
        return {i, j, l};
    }

    size_t ij2k(size_t i, size_t j, size_t n) {
        if (i == j) {
            throw std::runtime_error("i and j cannot be equal");
        }
        if (i > j) {
            std::swap(i, j);
        }
        return n * (n - 1) / 2 - (n - i) * (n - i - 1) / 2 + j - i - 1;
    }
}