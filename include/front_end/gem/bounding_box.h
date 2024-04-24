/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_BOUNDING_BOX_H
#define SRC_BOUNDING_BOX_H

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


namespace BBoxUtils
{

    struct IndexedPoint2d
    {
        size_t index;
        Eigen::Vector2d point;
        IndexedPoint2d(size_t index, const Eigen::Vector2d &point)
                : index(index)
                , point(point)
        {

        }
    };

    struct RotatedRect
    {
        Eigen::Matrix3d basis_; // basis of the rectangle, Rwb
        Eigen::Vector3d size_; // size of the rectangle
        double area, score;
        Eigen::Vector3d bottomLeft;
        Eigen::Vector3d topRight;

        RotatedRect() : area(std::numeric_limits<double>::max()) {}

        RotatedRect(const Eigen::Matrix3Xd &matrix, const Eigen::Matrix3d &basis, double degrees)
        {
            Eigen::Matrix3d R_12 = Eigen::AngleAxisd(degrees * 3.14159 / 180., Eigen::Vector3d::UnitZ()).toRotationMatrix();

            basis_ = basis * R_12;
            Eigen::Matrix3Xd newMatrix = basis_.transpose() * matrix; // transform the points from world frame to local frame
            topRight = newMatrix.rowwise().maxCoeff();
            bottomLeft = newMatrix.rowwise().minCoeff();
            size_(0) = topRight(0) - bottomLeft(0);
            size_(1) = topRight(1) - bottomLeft(1);
            size_(2) = topRight(2) - bottomLeft(2);
            area = size_(0) * size_(1);
            score = area; // smaller is better
            //score = 0;
            // evaluate how tight the bounding box is
            for (int i = 0;i < newMatrix.cols(); i++){
                const Eigen::Vector2d &point = newMatrix.block<2, 1>(0, i);
                const Eigen::Vector2d &distLeftBottom = (point - bottomLeft.block<2, 1>(0, 0)).cwiseAbs();
                const Eigen::Vector2d &distRightTop = (point - topRight.block<2, 1>(0, 0)).cwiseAbs();
                double dist = std::min(std::min(distLeftBottom(0), distLeftBottom(1)), std::min(distRightTop(0), distRightTop(1)));
                if (distLeftBottom(0) < 0 || distLeftBottom(1) < 0 || distLeftBottom(0) > size_(0) || distLeftBottom(1) > size_(1)){
                    score += dist * 1.414;
                } else{
                    score += dist;
                }
            }
        }

        void recalc(const float& minZ, const float& maxZ){
            size_(2) = maxZ - minZ;
            topRight(2) = maxZ;
            bottomLeft(2) = minZ;
        }
    };
    
    /**
     * @brief Given a R3 vector (v1), calculate two orthogonal vectors (v2 and v3) to
     * form a orthogonal basis
     * @param normal
     *      Vector to which the other two orthogonal vectors will be calculated upon
     * @param basis1
     *      An orthogonal vector to normal and basis2
     * @param basis2
     *      An orthogonal vector to normal and basis1
     */
    inline Eigen::Matrix3d orthogonalBasis(Eigen::Vector3d &v2, Eigen::Vector3d &v3, const Eigen::Vector3d &normal)
    {
        v2 = Eigen::Vector3d(normal.y() - normal.z(), -normal.x(), normal.x());
        v2 = v2.normalized();
        v3 = normal.cross(v2);
        v3 = v3.normalized();
        Eigen::Matrix3d basis;
        basis << v2, v3, normal;
        return basis;
    }

    inline float convexHullCross(const IndexedPoint2d *o, const IndexedPoint2d *a, const IndexedPoint2d *b)
    {
        return (a->point.x() - o->point.x()) * (b->point.y() - o->point.y()) -
               (a->point.y() - o->point.y()) * (b->point.x() - o->point.x());
    }
    
    // monotone chain algorithm for convex hull
    // https://en.wikipedia.org/wiki/Convex_hull_algorithms#Algorithms
    // https://github.com/MiguelVieira/ConvexHull2D/blob/master/ConvexHull.cpp#L119
    // https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C++
    inline void convexHull(const std::vector<Eigen::Vector2d> &points, std::vector<size_t> &indices)
    {
        if (points.size() < 3)
        {
            for (size_t i = 0; i < points.size(); i++)
            {
                indices.push_back(i);
            }
            return;
        }
        std::vector<IndexedPoint2d*> indexedPoints(points.size());
        for (size_t i = 0; i < indexedPoints.size(); i++)
        {
            indexedPoints[i] = new IndexedPoint2d(i, points[i]);
        }
        std::sort(indexedPoints.begin(), indexedPoints.end(), [](const IndexedPoint2d *a, const IndexedPoint2d *b) {
            return a->point.x() < b->point.x() || (a->point.x() == b->point.x() && a->point.y() < b->point.y());
        });
        std::vector<IndexedPoint2d*> H(2*points.size());
        size_t k = 0;

        // Build lower hull
        for (size_t i = 0; i < indexedPoints.size(); i++)
        {
            while (k >= 2 && convexHullCross(H[k-2], H[k-1], indexedPoints[i]) <= 0) k--;
            H[k++] = indexedPoints[i];
        }

        // Build upper hull
        for (size_t i = indexedPoints.size()-1, t = k+1; i > 0; --i)
        {
            while (k >= t && convexHullCross(H[k-2], H[k-1], indexedPoints[i-1]) <= 0) k--;
            H[k++] = indexedPoints[i-1];
        }

        H.resize(k-1);
        indices = std::vector<size_t>(H.size());
        for (size_t i = 0; i < H.size(); i++)
        {
            indices[i] = H[i]->index;
        }

        for (size_t i = 0; i < indexedPoints.size(); i++)
        {
            delete indexedPoints[i];
        }
    }
    
    // determine the boundary of the plane
    template <typename PointT>
    RotatedRect fittingBoundary(const pcl::PointCloud<PointT>& cloud,
                                   const Eigen::Vector3d& normal)
    {
        Eigen::Vector3d basisU, basisV;
        Eigen::Matrix3f basis = orthogonalBasis(basisU, basisV, normal).cast<float>();
        std::vector<Eigen::Vector2d> projectedPoints(cloud.points.size());
        float minZ = std::numeric_limits<float>::max();
        float maxZ = -std::numeric_limits<float>::max();
        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            Eigen::Vector3f point_body = basis.transpose() * cloud.points[i].getVector3fMap();
            projectedPoints[i] = point_body.head<2>().cast<double>();
            minZ = std::min(minZ, point_body.z());
            maxZ = std::max(maxZ, point_body.z());
        }
        std::vector<size_t> outlier; //the point indices that are on the boundary
        convexHull(projectedPoints, outlier);
        Eigen::Matrix3Xd matrix(3, outlier.size());
        for (size_t i = 0; i < outlier.size(); i++)
            matrix.col(i) = cloud.points[outlier[i]].getVector3fMap().template cast<double>();
        
        double minAngle = 0;
        double maxAngle = 90;
        // binary search
        while (maxAngle - minAngle > 1)
        {
            double mid = (maxAngle + minAngle) / 2;
            double left = (minAngle + mid) / 2;
            double right = (maxAngle + mid) / 2;
            RotatedRect leftRect(matrix, basis.cast<double>(), left);
            RotatedRect rightRect(matrix, basis.cast<double>(), right);
            maxAngle = leftRect.score < rightRect.score ? mid : maxAngle;
            minAngle = leftRect.score < rightRect.score ? minAngle : mid;
        }

        // find the rectangle with the largest area
        RotatedRect rect = RotatedRect(matrix, basis.cast<double>(), (minAngle + maxAngle) / 2);
        rect.recalc(minZ, maxZ);
        return rect;
    }

};

#endif //SRC_BOUNDING_BOX_H
