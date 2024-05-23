#include <iostream>
#include <string>
#include <Eigen/Core>
#include <signal.h>
#include "datasets/datasets_init.h"
#include "utils/evaluation.h"

using namespace std;
using namespace g3reg;

void signal_callback_handler(int signum);

int main(int argc, char **argv) {

    if (argc != 3) {
        std::cout << "Usage: reg_bm config_file test_file" << std::endl;
        return -1;
    }
    std::string config_path = config.project_path + "/" + argv[1];
    InitGLOG(config_path, argv);
    LOG(INFO) << "Test file: " << config.test_file;
    config.load_config(config_path, argv);

    std::map<int, std::map<std::string, std::vector<double>>> result_map;
    DataLoader::Ptr dataloader_ptr = CreateDataLoader();
    auto &items = dataloader_ptr->items;

    // downsample items by 10
//    std::vector<std::tuple<int, int, int, Eigen::Matrix4d>> pairs_downsampled;
//    for (int i = 0; i < items.size(); i += 1000) {
//        pairs_downsampled.push_back(items[i]);
//    }
//    items = pairs_downsampled;

    int total = 0;
    double precision = 0.0, time = 0.0, tp = 0.0, success1 = 0.0, success3 = 0.0, assoc_num = 0.0;
//#pragma omp parallel for
    for (auto &item: items) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud = dataloader_ptr->GetCloud(config.dataset_root, item.seq,
                                                                                 item.src_idx);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud = dataloader_ptr->GetCloud(config.dataset_root, item.seq,
                                                                                 item.tgt_idx);
        FeatureMetric metric = FrontEndEval(src_cloud, tgt_cloud, std::make_tuple(item.seq, item.src_idx, item.tgt_idx),
                                            item.pose);

        signal(SIGINT, signal_callback_handler);
//#pragma omp critical
        {
            precision += metric.precision;
            time += metric.time;
            tp += metric.tp;
            assoc_num += metric.assoc_num;
            if (metric.tp >= 3) success3 += 100;
            if (metric.tp >= 1) success1 += 100;
            total++;
            LOG(INFO) << std::fixed << std::setprecision(3) << "seq: " << item.seq << " " << total << "/"
                      << items.size()
                      << ", inlier ratio: " << metric.precision << "%, TP: " << metric.tp << " assoc: "
                      << metric.assoc_num << " t: "
                      << metric.time << "ms" << " succ(1/3): " << success1 / total << "/" << success3 / total;
        };
    }
    LOG(INFO) << "Final result: " << std::fixed << std::setprecision(3) << "inlier ratio: " << precision / total
              << "%, TP: " << tp / total << " assoc: " << assoc_num / total << " t: "
              << time / total << "ms" << " succ(1/3): " << success1 / total << "/" << success3 / total;
    return 0;
}


void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}