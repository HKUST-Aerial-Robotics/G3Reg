#include "utils/config.h"
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "datasets/datasets_init.h"
#include "back_end/reglib.h"
#include "global_definition/global_definition.h"
#include <pcl/common/transforms.h>
#include "robot_utils/file_manager.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace clique_solver;
using namespace g3reg;


template<typename PointT>
void visualizeCloud(typename pcl::PointCloud<PointT>::Ptr source, typename pcl::PointCloud<PointT>::Ptr target) {
    pcl::visualization::PCLVisualizer viewer("");
    viewer.setBackgroundColor(255, 255, 255);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> sourceColor(source, 255, 180, 0);
    viewer.addPointCloud<PointT>(source, sourceColor, "source");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> targetColor(target, 0, 166, 237);
    viewer.addPointCloud<PointT>(target, targetColor, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    // set camera position at the center of the point source
    viewer.setCameraPosition(source->points[source->size() / 2].x,
                             source->points[source->size() / 2].y,
                             source->points[source->size() / 2].z,
                             0, 0, 1);
    viewer.spin();
    viewer.close();
}

int main(int argc, char **argv) {
    if (argc < 4) {
        std::cout << "Usage: reg_bm config_file pcd1 pcd2" << std::endl;
        return -1;
    }
    std::string config_path = config.project_path + "/" + argv[1];
    InitGLOG(config_path, argv);
    config.load_config(config_path, argv);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(std::string(argv[2]), *source) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file source.pcd \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(std::string(argv[3]), *target) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file target.pcd \n");
        return (-1);
    }

    FRGresult solution = g3reg::GlobalRegistration(source, target);
    Eigen::Matrix4d tf = solution.tf;

    std::string log_dir = g3reg::WORK_SPACE_PATH + "/Log/demo/";
    FileManager::CreateDirectory(log_dir);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *src_transformed, tf);
    pcl::io::savePCDFileASCII(log_dir + "source_transformed.pcd", *src_transformed);
    pcl::io::savePCDFileASCII(log_dir + "target.pcd", *target);
    LOG(INFO) << "Estimated transformation:";
    LOG(INFO) << tf;

    visualizeCloud<pcl::PointXYZ>(src_transformed, target);

    return 0;
}