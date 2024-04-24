/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
//
#include "front_end/fpfh_utils.h"
#include <dataset/kitti_utils.h>
#include "front_end/gem/downsample.h"
#include <pcl/common/transforms.h>

namespace fpfh{

    void Match(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                   std::vector<std::pair<int, int>> &corres) {
        // KITTI parameters for FPFH while voxel downsampling resolution is 0.3
        double normal_radius = config::normal_radius;
        double fpfh_radius = config::fpfh_radius;

        // Compute FPFH
        teaser::FPFHEstimation fpfh;
        teaser::FPFHCloudPtr obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, normal_radius, fpfh_radius);
        teaser::FPFHCloudPtr scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, normal_radius, fpfh_radius);
        teaser::Matcher matcher;
        corres = matcher.calculateCorrespondences(
                src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, true, true, false, 0.95);
    }

    clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                        std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes) {
        double tSrc, tTgt;
        pcl::PointCloud<pcl::PointXYZ> srcGround;
        pcl::PointCloud<pcl::PointXYZ> tgtGround;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcNonground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTgtNonground(new pcl::PointCloud<pcl::PointXYZ>);
        travel::estimateGround(*(src_cloud), srcGround, *ptrSrcNonground, tSrc);
        travel::estimateGround(*(tgt_cloud), tgtGround, *ptrTgtNonground, tTgt);

        // voxelize
        const double voxel_size = 0.5;
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ds(new pcl::PointCloud<pcl::PointXYZ>);
        voxelize(ptrSrcNonground, src_ds, voxel_size);
        voxelize(ptrTgtNonground, tgt_ds, voxel_size);

        std::vector<std::pair<int, int>> corres;
        Match(src_ds, tgt_ds, corres);
        clique_solver::Association assoc = clique_solver::Association::Zero(corres.size(), 2);
        for (int i = 0; i < corres.size(); i++) {
            assoc(i, 0) = corres[i].first;
            assoc(i, 1) = corres[i].second;
        }

        // construct the matching node pairs
        src_nodes.clear();
        tgt_nodes.clear();
        src_nodes.reserve(src_ds->size());
        tgt_nodes.reserve(tgt_ds->size());

        for (int i = 0; i < src_ds->size(); i++) {
            const Eigen::Vector3d& center = src_ds->at(i).getVector3fMap().cast<double>();
            src_nodes.push_back(clique_solver::create_vertex(center, config::vertex_info));
        }

        for (int i = 0; i < tgt_ds->size(); i++) {
            const Eigen::Vector3d& center = tgt_ds->at(i).getVector3fMap().cast<double>();
            tgt_nodes.push_back(clique_solver::create_vertex(center, config::vertex_info));
        }
        return assoc;
    }
}

namespace iss_fpfh{
	
	void fpfhComputation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution, pcl::PointIndicesPtr iss_Idx, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_out)
	{
		//compute normal
		pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(3 * resolution);
		ne.compute(*normal);
		
		//compute fpfh using normals
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud(cloud);
		fpfh_est.setInputNormals(normal);
		fpfh_est.setSearchMethod(tree);
		fpfh_est.setRadiusSearch(8 * resolution);
		fpfh_est.setNumberOfThreads(16);
		fpfh_est.setIndices(iss_Idx);
		fpfh_est.compute(*fpfh_out);
	}
	
	void correspondenceSearching(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfht, pcl::Correspondences & corr, int max_corr, std::vector<int>& corr_NOs, std::vector<int>& corr_NOt)
	{
		int n = std::min(max_corr, (int)fpfht->size()); //maximum number of correspondences to find for each source point
		corr.clear();
		corr_NOs.assign(fpfhs->size(), 0);
		corr_NOt.assign(fpfht->size(), 0);
		// Use a KdTree to search for the nearest matches in feature space
		pcl::KdTreeFLANN<pcl::FPFHSignature33> treeS;
		treeS.setInputCloud(fpfhs);
		pcl::KdTreeFLANN<pcl::FPFHSignature33> treeT;
		treeT.setInputCloud(fpfht);
		for (size_t i = 0; i < fpfhs->size(); i++) {
			std::vector<int> corrIdxTmp(n);
			std::vector<float> corrDisTmp(n);
			//find the best n matches in target fpfh
			treeT.nearestKSearch(*fpfhs, i, n, corrIdxTmp, corrDisTmp);
			for (size_t j = 0; j < corrIdxTmp.size(); j++) {
				bool removeFlag = true;
				int searchIdx = corrIdxTmp[j];
				std::vector<int> corrIdxTmpT(n);
				std::vector<float> corrDisTmpT(n);
				treeS.nearestKSearch(*fpfht, searchIdx, n, corrIdxTmpT, corrDisTmpT);
				for (size_t k = 0; k < n; k++) {
					if (corrIdxTmpT.data()[k] == i) {
						removeFlag = false;
						break;
					}
				}
				if (removeFlag == false) {
					pcl::Correspondence corrTabTmp;
					corrTabTmp.index_query = i;
					corrTabTmp.index_match = corrIdxTmp[j];
					corrTabTmp.distance = corrDisTmp[j];
					corr.push_back(corrTabTmp);
					corr_NOs[i]++;
					corr_NOt[corrIdxTmp[j]]++;
				}
			}
		}
	}
	
	void Match(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
			   std::vector<std::pair<int, int>> &corres) {
		
		// ISS detector parameters
		pcl::PointCloud<pcl::PointXYZ>::Ptr issS(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr issT(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndicesPtr iss_IdxS(new pcl::PointIndices);
		pcl::PointIndicesPtr iss_IdxT(new pcl::PointIndices);
		pcl::issKeyPointExtration(src_cloud, issS, iss_IdxS, config::ds_resolution);
		pcl::issKeyPointExtration(tgt_cloud, issT, iss_IdxT, config::ds_resolution);
		
		// FPFH descriptor parameters
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhS(new pcl::PointCloud<pcl::FPFHSignature33>());
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhT(new pcl::PointCloud<pcl::FPFHSignature33>());
		fpfhComputation(src_cloud, config::ds_resolution, iss_IdxS, fpfhS);
		fpfhComputation(tgt_cloud, config::ds_resolution, iss_IdxT, fpfhT);
		
		// Find correspondences
		std::vector<int> corr_NOS, corr_NOT;
		pcl::CorrespondencesPtr corr(new pcl::Correspondences());
		int max_corr = 5;
		correspondenceSearching(fpfhS, fpfhT, *corr, max_corr, corr_NOS, corr_NOT);
		
		//write corres
		corres.reserve(corr->size());
		for (size_t i = 0; i < corr->size(); i++) {
			std::pair<int, int> corresTmp;
			corresTmp.first = corr->at(i).index_query;
			corresTmp.second = corr->at(i).index_match;
			corres.push_back(corresTmp);
		}
	}
	
	clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
										std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes) {
		double tSrc, tTgt;
		pcl::PointCloud<pcl::PointXYZ> srcGround;
		pcl::PointCloud<pcl::PointXYZ> tgtGround;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcNonground(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTgtNonground(new pcl::PointCloud<pcl::PointXYZ>);
		travel::estimateGround(*(src_cloud), srcGround, *ptrSrcNonground, tSrc);
		travel::estimateGround(*(tgt_cloud), tgtGround, *ptrTgtNonground, tTgt);
		
		// voxelize
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_ds(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ds(new pcl::PointCloud<pcl::PointXYZ>);
		voxelize(ptrSrcNonground, src_ds, config::ds_resolution);
		voxelize(ptrTgtNonground, tgt_ds, config::ds_resolution);
		
		std::vector<std::pair<int, int>> corres;
		Match(src_ds, tgt_ds, corres);
		clique_solver::Association assoc = clique_solver::Association::Zero(corres.size(), 2);
		for (int i = 0; i < corres.size(); i++) {
			assoc(i, 0) = corres[i].first;
			assoc(i, 1) = corres[i].second;
		}
		
		// construct the matching node pairs
		src_nodes.clear();
		tgt_nodes.clear();
		src_nodes.reserve(src_ds->size());
		tgt_nodes.reserve(tgt_ds->size());
		
		for (int i = 0; i < src_ds->size(); i++) {
			const Eigen::Vector3d& center = src_ds->at(i).getVector3fMap().cast<double>();
			src_nodes.push_back(clique_solver::create_vertex(center, config::vertex_info));
		}
		
		for (int i = 0; i < tgt_ds->size(); i++) {
			const Eigen::Vector3d& center = tgt_ds->at(i).getVector3fMap().cast<double>();
			tgt_nodes.push_back(clique_solver::create_vertex(center, config::vertex_info));
		}
		return assoc;
	}
}