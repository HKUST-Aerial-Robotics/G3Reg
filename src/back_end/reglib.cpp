/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "back_end/reglib.h"
#include "utils/evaluation.h"
#include "front_end/gem/ellipsoid.h"
#include "front_end/fpfh_utils.h"
#include "front_end/fcgf.h"
#include "back_end/pagor/pagor.h"
#include "back_end/ransac/ransac.h"
#include "back_end/mac3d/mac_reg.h"

using namespace std;
using namespace clique_solver;

namespace g3reg{ //fast and robust global registration
    
    FRGresult GlobalRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                 std::tuple<int, int, int> pair_info){

        FRGresult result;

        std::vector<GraphVertex::Ptr> src_nodes, tgt_nodes;
        std::vector<g3reg::QuadricFeature::Ptr> src_features, dst_features;
        Association A;
        g3reg::EllipsoidMatcher matcher(src_cloud, tgt_cloud);
        robot_utils::TicToc front_end_timer, timer;
        if (config::front_end == "gem"){
            A = std::move(matcher.matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
        } else if (config::front_end == "fpfh"){
            A = std::move(fpfh::matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
        } else if (config::front_end == "iss_fpfh"){
			A = std::move(iss_fpfh::matching(src_cloud, tgt_cloud, src_nodes, tgt_nodes));
		} else if (config::front_end == "fcgf"){
            A = std::move(fcgf::matching(pair_info, src_nodes, tgt_nodes));
        } else if (config::front_end == "none"){
		}
        result.feature_time = front_end_timer.toc();
		
        if (config::back_end == "pagor"){
            pagor::solve(src_nodes, tgt_nodes, A, matcher, result);
        } else if (config::back_end == "ransac"){
            ransac::solve(src_nodes, tgt_nodes, A, result);
        } else if (config::back_end == "3dmac"){
            mac_reg::solve(src_nodes, tgt_nodes, A, result);
        } else {
            throw std::runtime_error("Unknown back end method");
        }

        result.total_time = timer.toc();
        return result;
    }
}