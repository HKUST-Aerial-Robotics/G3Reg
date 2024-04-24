/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_ELLIPSOID_H
#define SRC_ELLIPSOID_H
#include "front_end/graph_vertex.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "utils/config.h"
#include "front_end/gem/lineplane_extractor.h"
#include "utils/evaluation.h"

namespace g3reg {

//    template<typename PointT>
    class EllipsoidMatcher {

    public:
//        typedef boost::shared_ptr<pcl::PointCloud<PointT>> PointCloudPtr;
        EllipsoidMatcher(){
            src_ellipsoids_.clear();
            tgt_ellipsoids_.clear();
            src_ellipsoids_vec_.clear();
            tgt_ellipsoids_vec_.clear();
        }

        EllipsoidMatcher(pcl::PointCloud<pcl::PointXYZ>::Ptr srcPc,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr tgtPc){
            srcPc_ = srcPc;
            tgtPc_ = tgtPc;
            EllipsoidMatcher();
        }

        void extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud);

        void associateAdvanced();

        clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                            std::vector<clique_solver::GraphVertex::Ptr> &src_nodes,
                                            std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes);

        clique_solver::Association getAssociation(){
            return A_;
        }

        std::vector<g3reg::QuadricFeature::Ptr> getSrcEllipsoids(){
            return src_ellipsoids_;
        }

        std::vector<g3reg::QuadricFeature::Ptr> getTgtEllipsoids(){
            return tgt_ellipsoids_;
        }

        const VoxelMap& getSrcVoxels() const{
            return plc_src_.getVoxels();
        }

        const VoxelMap& getTgtVoxels() const{
            return plc_tgt_.getVoxels();
        }

        const pcl::PointCloud<pcl::PointXYZ>::Ptr& getSrcPc() const{
            return srcPc_;
        }

        const pcl::PointCloud<pcl::PointXYZ>::Ptr& getTgtPc() const{
            return tgtPc_;
        }

    private:

        void TopKEllipse(std::vector<g3reg::QuadricFeature::Ptr> &ellipsoids, int k);

        void TransformToEllipsoid(const FeatureSet& featureSet, std::vector<std::vector<QuadricFeature::Ptr>>& ellipsoids);

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr srcPc_, tgtPc_;
        g3reg::PLCExtractor plc_src_, plc_tgt_;
        FeatureSet src_features_, tgt_features_;
        std::vector<std::vector<g3reg::QuadricFeature::Ptr>> src_ellipsoids_vec_, tgt_ellipsoids_vec_;
        std::vector<g3reg::QuadricFeature::Ptr> src_ellipsoids_, tgt_ellipsoids_;
        clique_solver::Association A_;
    };

    std::vector<g3reg::QuadricFeature::Ptr> transformQuadric(const std::vector<g3reg::QuadricFeature::Ptr>& quadric_vec, const Eigen::Matrix4d& T);
}
#endif //SRC_ELLIPSOID_H
