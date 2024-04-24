/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include <Eigen/Core>
#include <queue>
#include <vector>
#include "front_end/gem/gem_matching.h"
#include "front_end/gem/ellipsoid.h"

namespace g3reg{

// Define a comparison struct for the priority queue
    struct Compare {
        bool operator()(const std::pair<int, double>& p1, const std::pair<int, double>& p2) {
            return p1.second < p2.second;
        }
    };

    std::vector<std::pair<int, int>> MatchingGEMs(std::vector<GEM::Ptr> &src, std::vector<GEM::Ptr> &tgt, int K){
        std::vector<std::vector<std::pair<int, double>>> src_to_tgt(src.size()), tgt_to_src(tgt.size());
        int K_src = std::min(K, static_cast<int>(tgt.size()));
        int K_tgt = std::min(K, static_cast<int>(src.size()));

        // Compute top K_src matches from src to tgt
        for (int i = 0; i < src.size(); ++i) {
            std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, Compare> pq;
            for (int j = 0; j < tgt.size(); ++j) {
                double sim = src[i]->similarity(*tgt[j]);
                if (pq.size() < K_src) {
                    pq.push(std::make_pair(j, sim));
                } else if (sim < pq.top().second) {
                    pq.pop();
                    pq.push(std::make_pair(j, sim));
                }
            }
            while (!pq.empty()) {
                src_to_tgt[i].push_back(pq.top());
                pq.pop();
            }
        }

        // Compute top K_tgt matches from tgt to src
        for (int j = 0; j < tgt.size(); ++j) {
            std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, Compare> pq;
            for (int i = 0; i < src.size(); ++i) {
                double sim = tgt[j]->similarity(*src[i]);
                if (pq.size() < K_tgt) {
                    pq.push(std::make_pair(i, sim));
                } else if (sim < pq.top().second) {
                    pq.pop();
                    pq.push(std::make_pair(i, sim));
                }
            }
            while (!pq.empty()) {
                tgt_to_src[j].push_back(pq.top());
                pq.pop();
            }
        }

        // Only keep mutual matches
        std::vector<std::pair<int, int>> corres;
        for (int i = 0; i < src.size(); ++i) {
            for (const auto& match : src_to_tgt[i]) {
                int j = match.first;
                auto it = std::find_if(tgt_to_src[j].begin(), tgt_to_src[j].end(), [&](const std::pair<int, double>& p) {
                    return p.first == i;
                });
                if (it != tgt_to_src[j].end()) {
                    corres.push_back(std::make_pair(i, j));
                }
            }
        }

        return corres;
    }
}