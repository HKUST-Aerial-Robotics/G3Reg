/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_DATASETS_INIT_H
#define SRC_DATASETS_INIT_H

#include "datasets/kitti_loader.h"
#include "datasets/apollo_loader.h"
#include "datasets/kitti360_loader.h"
#include "datasets/hit_loader.h"

inline DataLoader::Ptr CreateDataLoader() {
    DataLoader::Ptr data_loader;
    // use Lower case letters
    std::string dataset_name = config::dataset_name;
    std::transform(dataset_name.begin(), dataset_name.end(), dataset_name.begin(), ::tolower);
    if (dataset_name == "kitti") {
        data_loader = std::make_shared<KittiLoader>();
    } else if (dataset_name == "apollo") {
        data_loader = std::make_shared<ApolloLoader>();
    } else if (dataset_name == "kitti360") {
        data_loader = std::make_shared<KITTI360Loader>();
    } else if (dataset_name == "hit_ms"){
        data_loader = std::make_shared<HITLoader>();
    } else {
        LOG(ERROR) << "Unknown dataset type: " << dataset_name;
    }
    return data_loader;
}

#endif //SRC_DATASETS_INIT_H
