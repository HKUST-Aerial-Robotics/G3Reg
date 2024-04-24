/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_ALGORITHMS_H
#define SRC_ALGORITHMS_H

#include <algorithm>
#include <vector>
#include <iostream>

namespace robot_utils{

    template<typename T>
    bool are_vectors_equal(const std::vector<T>& v1, const std::vector<T>& v2) {
        if (v1.size() != v2.size()) {
            return false;
        }

        std::vector<T> sorted_v1(v1);
        std::vector<T> sorted_v2(v2);

        std::sort(sorted_v1.begin(), sorted_v1.end());
        std::sort(sorted_v2.begin(), sorted_v2.end());

        return sorted_v1 == sorted_v2;
    }

    template<typename T>
    std::vector<std::pair<int, int>> buildMutualTopKAssociations(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& costs, int K) {
//        smaller is better
        int numRows = costs.rows();
        int numCols = costs.cols();

        // Adjust K if the number of rows or columns is less than K
        int adjustedK = std::min({K, numRows, numCols});

        // Step 1: Sort the matrix elements by their scores
        using Element = std::pair<T, std::pair<int, int>>;
        std::priority_queue<Element, std::vector<Element>, std::greater<Element>> minHeap;
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < numCols; ++j) {
                minHeap.push({costs(i, j), {i, j}});
            }
        }

        // Step 2: Keep track of the source and target indices for each sorted element
        std::vector<std::vector<int>> topKSource(numRows, std::vector<int>(adjustedK, -1));
        std::vector<std::vector<int>> topKTarget(numCols, std::vector<int>(adjustedK, -1));

        while (!minHeap.empty()) {
            Element element = minHeap.top();
            minHeap.pop();
            int source = element.second.first;
            int target = element.second.second;

            // Check if the source and target are already in the top-K lists
            bool inTopKSource = std::find(topKSource[source].begin(), topKSource[source].end(), target) != topKSource[source].end();
            bool inTopKTarget = std::find(topKTarget[target].begin(), topKTarget[target].end(), source) != topKTarget[target].end();

            if (!inTopKSource && !inTopKTarget) {
                for (int k = 0; k < adjustedK; ++k) {
                    if (topKSource[source][k] == -1) {
                        topKSource[source][k] = target;
                        break;
                    }
                }
                for (int k = 0; k < adjustedK; ++k) {
                    if (topKTarget[target][k] == -1) {
                        topKTarget[target][k] = source;
                        break;
                    }
                }
            }
        }

        // Step 3: Build associations
        std::vector<std::pair<int, int>> associations;
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < adjustedK; ++j) {
                int target = topKSource[i][j];
                if (target != -1) {
                    int sourceIndex = std::find(topKTarget[target].begin(), topKTarget[target].end(), i) - topKTarget[target].begin();
                    if (sourceIndex < adjustedK) {
                        associations.push_back({i, target});
                    }
                }
            }
        }

        return associations;
    }
}

#endif //SRC_ALGORITHMS_H
