/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_DATALOADER_H
#define SRC_DATALOADER_H
#include "utils/config.h"

class DataLoader {
public:
    struct Item {
        int seq, src_idx, seq_db, tgt_idx;
        Eigen::Matrix4d pose;
        double overlap;
    };
    std::vector<Item> items;
    typedef std::shared_ptr<DataLoader> Ptr;
	std::map<int, std::map<int, Eigen::Matrix4d>> lidar_poses; // seq, frame_id, pose
public:
    DataLoader() {
        std::string test_file = config::project_path + "/" + config::test_file;
        LOG(INFO) << "Reading test file: " << test_file;
        LoadTestFile(test_file);
    }
    
    void LoadTestFile(std::string test_file) {
        /**test.txt
         * seq i seq_db j mot1 mot2 mot3 mot4 mot5 mot6 mot7 mot8 mot9 mot10 mot11 mot12 mot13 mot14 mot15 overlap
         * */
        std::ifstream file(test_file);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file");
        }
        // skip the first line
        std::string line;
        std::getline(file, line);

        int seq, i, seq_db, j;
        double overlap;
        while (std::getline(file, line)) {
            std::vector<float> rowNumbers;
            std::istringstream iss(line);

            float num;
            while (iss >> num) {
                rowNumbers.push_back(num);
            }
            seq = rowNumbers[0];
            i = rowNumbers[1];
            seq_db = rowNumbers[2];
            j = rowNumbers[3];
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose << rowNumbers[4], rowNumbers[5], rowNumbers[6], rowNumbers[7],
                    rowNumbers[8], rowNumbers[9], rowNumbers[10], rowNumbers[11],
                    rowNumbers[12], rowNumbers[13], rowNumbers[14], rowNumbers[15],
                    rowNumbers[16], rowNumbers[17], rowNumbers[18], rowNumbers[19];
            if (rowNumbers.size() == 21) {
                overlap = rowNumbers[20];
            } else {
                overlap = 1.0;
            }
            items.push_back({seq, i, seq_db, j, pose, overlap});
        }
        LOG(INFO) << "Loading " << items.size() << " items";
        file.close(); //关闭文件
    }
    
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string dataset_root, int seq, int i) = 0;

    virtual Eigen::Matrix4d getLidarPose(std::string dataset_root, int seq, int frame_id) = 0;
	
	virtual void LoadLiDARPoses(std::string dataset_root, int seq = -1) = 0;
};

#endif //SRC_DATALOADER_H
