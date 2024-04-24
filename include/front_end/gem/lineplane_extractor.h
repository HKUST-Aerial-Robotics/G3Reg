/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_LINEPLANE_EXTRACTOR_H
#define SRC_LINEPLANE_EXTRACTOR_H

#include <vector>
#include <memory>
#include <mutex>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

#include <pcl/segmentation/extract_clusters.h>
#include "dataset/kitti_utils.h"
#include "gemodel.h"
#include "voxel.h"
#include "front_end/gem/clustering.h"

namespace g3reg {

    typedef struct {
        std::vector<LineFeature::Ptr> lines;
        std::vector<SurfaceFeature::Ptr> planes;
        std::vector<ClusterFeature::Ptr> clusters;
    } FeatureSet;

    class PLCExtractor {
    public:
        typedef std::shared_ptr<PLCExtractor> Ptr;

        PLCExtractor() = default;

        ~PLCExtractor() = default;

        void ExtractFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &sem_cloud,
                            std::vector<LineFeature::Ptr> &line_features,
                            std::vector<SurfaceFeature::Ptr> &surface_features,
                            std::vector<ClusterFeature::Ptr> &cluster_features);

        void ExtractPole(std::vector<ClusterFeature::Ptr> &cluster_features,
                         std::vector<LineFeature::Ptr> &line_features);
	
		void ExtractPlanes(std::vector<ClusterFeature::Ptr> &cluster_features,
						   std::vector<SurfaceFeature::Ptr> &plane_features);

        void MergePlanes(std::vector<SurfaceFeature::Ptr> &surface_features);

        void FilterSurface(std::vector<SurfaceFeature::Ptr> &surface_features, int min_points);

        void reset();

        const VoxelMap& getVoxels() const{
            return voxel_map;
        }

    private:
        VoxelMap voxel_map;
    };
    
} // namespace g3reg

#endif //SRC_LINEPLANE_EXTRACTOR_H
