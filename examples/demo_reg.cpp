#include "utils/config.h"
#include <iostream>
#include <string>
#include <Eigen/Core>
#include "datasets/datasets_init.h"
#include "back_end/reglib.h"
#include "global_definition/global_definition.h"
#include <pcl/common/transforms.h>
#include "robot_utils/file_manager.h"

using namespace std;
using namespace clique_solver;

int main(int argc, char **argv) {
    if (argc < 4) {
        std::cout << "Usage: reg_bm config_file pcd1 pcd2" << std::endl;
        return -1;
    }
    std::string config_path = config::project_path + "/" + argv[1];
    InitGLOG(config_path, argv);
    config::readParameters(config_path, argv);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(argv[2]), *source) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file source.pcd \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(argv[3]), *target) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file target.pcd \n");
        return (-1);
    }

    FRGresult solution = g3reg::GlobalRegistration(source, target);
    Eigen::Matrix4d tf = solution.tf;

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_transformed (new pcl::PointCloud<pcl::PointXYZ>);
    std::string log_dir = g3reg::WORK_SPACE_PATH + "/Log/demo/";
    FileManager::CreateDirectory(log_dir);
    pcl::transformPointCloud(*source, *src_transformed, tf);
    pcl::io::savePCDFileASCII(log_dir + "source_transformed.pcd", *src_transformed);
    pcl::io::savePCDFileASCII(log_dir + "target.pcd", *target);
    LOG(INFO) << "Estimated transformation:";
    LOG(INFO) << tf;

    return 0;
}