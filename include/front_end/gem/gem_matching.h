/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_GEM_MATCHING_H
#define SRC_GEM_MATCHING_H

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "robot_utils/eigen_types.h"

namespace g3reg {
    typedef std::unordered_map<Eigen::Vector3i, int, robot_utils::hash_vec<3>> DescMap;

    class GEM {
    public:
        typedef boost::shared_ptr<GEM> Ptr;

        GEM(std::string metric) : metric(metric) {}

        GEM(const Eigen::VectorXd &desc, std::string metric = "2-norm") : desc(desc), metric(metric) {}

        void setDescMap(const DescMap &desc_map) {
            this->desc_map = desc_map;
        }

        DescMap desc_map;
        Eigen::VectorXd desc;
        std::string metric;

    public:
        double similarity(GEM &other) {
            // smaller is better
            if (metric == "2-norm")
                return (desc - other.desc).norm();
            else if (metric == "iou3d") {
                // desc is vector3d which is the size of the BBOX
                double w1 = desc(0), h1 = desc(1), l1 = desc(2);
                double w2 = other.desc(0), h2 = other.desc(1), l2 = other.desc(2);
                // both centers are at origin
                double dw = fabs(w1 - w2), dh = fabs(h1 - h2), dl = fabs(l1 - l2);
                double intersect = dw * dh * dl;
                double union_ = w1 * h1 * l1 + w2 * h2 * l2 - intersect;
                double iou = intersect / union_;
                return 1 - iou;
            } else if (metric == "hash_desc") {
                // typedef std::unordered_map<Eigen::Vector3i, int, robot_utils::hash_vec<3>> DescMap;
                DescMap &desc_map1 = this->desc_map;
                DescMap &desc_map2 = other.desc_map;
                int score = 0;
                // Iterate over each voxel in the first descriptor map
                for (const auto &pair1: desc_map1) {
                    // Now we create a 3x3x3 cube around the current voxel
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dz = -1; dz <= 1; dz++) {
                                Eigen::Vector3i neighbor(pair1.first[0] + dx, pair1.first[1] + dy, pair1.first[2] + dz);

                                // If the neighboring voxel is also in desc_map2, increment the score
                                if (desc_map2.find(neighbor) != desc_map2.end()) {
                                    score++;
                                }
                            }
                        }
                    }

//                    if (desc_map2.find(pair1.first) != desc_map2.end()) {
//                        score++;
//                    }
                }
                return 1 / (1 + score);
            } else {
                throw std::runtime_error("Unknown metric");
            }
        }
    };

    std::vector<std::pair<int, int>> MatchingGEMs(std::vector<GEM::Ptr> &src, std::vector<GEM::Ptr> &tgt, int K);
}


#endif //SRC_GEM_MATCHING_H
