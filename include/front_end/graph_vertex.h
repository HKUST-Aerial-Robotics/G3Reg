/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_GRAPH_VERTEX_H
#define SRC_GRAPH_VERTEX_H

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

namespace clique_solver {

    enum class VertexType {
        DEFAULT = -1,
        GAUSSIAN = 0,
        POINT = 1,
        POINT_RATIO = 2,
        ELLIPSE = 3,
    };

    class VertexInfo {
    public:
        VertexType type;
        double prior_bound = -1.0;
        std::vector<double> noise_bound_vec;
    };

    class GraphVertex {

    public:
        using Ptr = boost::shared_ptr<GraphVertex>;

        GraphVertex() = default;

        GraphVertex(const Eigen::Vector3d &centroid, const VertexInfo &vertex_info) : centroid(centroid),
                                                                                      vertex_info_(vertex_info) {
            covariance = Eigen::Matrix3d::Identity();
        }

        GraphVertex(const Eigen::Vector3d &centroid, const Eigen::Matrix3d &covariance,
                    const VertexInfo &vertex_info) : centroid(centroid), covariance(covariance),
                                                     vertex_info_(vertex_info) {}

        ~GraphVertex() = default;

        double norm() const { return centroid.norm(); }

        double m_dist() const { return centroid.transpose() * covariance.inverse() * centroid; }

        int num_graphs() const { return vertex_info_.noise_bound_vec.size(); }

        int type() const { return static_cast<int>(vertex_info_.type); }

        virtual GraphVertex::Ptr operator-(const GraphVertex &other) const {
            return GraphVertex::Ptr(new GraphVertex());
        }

        virtual Eigen::VectorXd consistent(const GraphVertex &other) {
            return Eigen::VectorXd::Zero(1, 1);
        }

    public:
        Eigen::Vector3d centroid;
        Eigen::Matrix3d covariance;
        VertexInfo vertex_info_;
    };

    class GaussianVertex : public GraphVertex {
    public:
        using Ptr = std::shared_ptr<GaussianVertex>;

        GaussianVertex() = default;

        ~GaussianVertex() = default;

        GaussianVertex(const Eigen::Vector3d &centroid, const Eigen::Matrix3d &covariance,
                       const VertexInfo &vertex_info) : GraphVertex(centroid, covariance, vertex_info) {}

        GraphVertex::Ptr operator-(const GraphVertex &other) const override;

        Eigen::VectorXd consistent(const GraphVertex &other) override;

        double wasserstein_distance(const GraphVertex &other);
    };

    class EllipseVertex : public GraphVertex {
    public:
        using Ptr = std::shared_ptr<EllipseVertex>;

        EllipseVertex() = default;

        ~EllipseVertex() = default;

        EllipseVertex(const Eigen::Vector3d &centroid, const Eigen::Matrix3d &covariance,
                      const VertexInfo &vertex_info) : GraphVertex(centroid, covariance, vertex_info) {}

        GraphVertex::Ptr operator-(const GraphVertex &other) const override;

        Eigen::VectorXd consistent(const GraphVertex &other) override;

        double upper_rho(const Eigen::Matrix3d &cov);
    };

    class PointVertex : public GraphVertex {
    public:
        using Ptr = std::shared_ptr<PointVertex>;

        PointVertex() = default;

        ~PointVertex() = default;

        PointVertex(const Eigen::Vector3d &centroid, const VertexInfo &vertex_info) : GraphVertex(centroid,
                                                                                                  vertex_info) {}

        GraphVertex::Ptr operator-(const GraphVertex &other) const override;

        Eigen::VectorXd consistent(const GraphVertex &other) override;
    };

    class PointRatioVertex : public GraphVertex {
    public:
        using Ptr = std::shared_ptr<PointRatioVertex>;

        PointRatioVertex() = default;

        ~PointRatioVertex() = default;

        PointRatioVertex(const Eigen::Vector3d &centroid, const VertexInfo &vertex_info) : GraphVertex(centroid,
                                                                                                       vertex_info) {}

        GraphVertex::Ptr operator-(const GraphVertex &other) const override;

        Eigen::VectorXd consistent(const GraphVertex &other) override;
    };

    clique_solver::GraphVertex::Ptr create_vertex(const Eigen::Vector3d &center, const VertexInfo &vertex_info,
                                                  const Eigen::Matrix3d &cov = Eigen::Matrix3d::Identity(),
                                                  const double &prior_bound = -1);

    std::tuple<size_t, size_t> k2ij(size_t k, size_t n);

    std::tuple<size_t, size_t, size_t> k2ijl(size_t k, size_t N, const std::vector<size_t> &indices);

    size_t ij2k(size_t i, size_t j, size_t n);
}

#endif //SRC_GRAPH_VERTEX_H
