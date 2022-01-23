/*
 * grid_map_pcl_from_pointcloud2.cpp
 *
 *  Created on: Nov 7, 2021
 *      Author: Yibin Li
 */

#include <iostream>

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

#include "opencv2/core.hpp"
//#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"


#include "sensor_msgs/PointCloud2.h"


namespace gm = ::grid_map::grid_map_pcl;

ros::Publisher gridMapPub;
ros::Subscriber sub;

grid_map::GridMapPclLoader gridMapPclLoader;
//ros::NodeHandle nh("~");

void pointcloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // maybe move all process into this subscriber callback function
    ROS_INFO("inside callback");
    ros::NodeHandle nh("~");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud_input);
    ROS_INFO("pointcloud2 to pcl pointcloud conversion succeed!");

    gridMapPclLoader.setInputCloud(pcl_cloud_input);
    gm::processPointcloud(&gridMapPclLoader, nh);

//    grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
    grid_map::GridMap originalMap = gridMapPclLoader.getGridMap();
    grid_map::GridMap gridMap;
    grid_map::GridMapCvProcessing::changeResolution(originalMap, gridMap, 0.001);
    gridMap.setFrameId(gm::getMapFrame(nh));

    std::vector<std::string> layer_name = gridMap.getLayers();
    for (auto &name : layer_name) {
        ROS_INFO("layer name %s", name.c_str());
    }

    std::string grid_map_col = std::to_string(gridMap.getSize()(1));
    std::string grid_map_row = std::to_string(gridMap.getSize()(0));

    ROS_INFO("grid map column size: %s", grid_map_col.c_str());
    ROS_INFO("grid map row size: %s", grid_map_row.c_str());

    grid_map::Matrix& m = gridMap.get("elevation");

//    namespace Eigen{
//        template<class Matrix>
//        void write_binary(const char* filename, const Matrix& matrix){
//            std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
//            typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
//            out.write((char*) (&rows), sizeof(typename Matrix::Index));
//            out.write((char*) (&cols), sizeof(typename Matrix::Index));
//            out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
//            out.close();
//        }
//        template<class Matrix>
//        void read_binary(const char* filename, Matrix& matrix){
//            std::ifstream in(filename, std::ios::in | std::ios::binary);
//            typename Matrix::Index rows=0, cols=0;
//            in.read((char*) (&rows),sizeof(typename Matrix::Index));
//            in.read((char*) (&cols),sizeof(typename Matrix::Index));
//            matrix.resize(rows, cols);
//            in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
//            in.close();
//        }
//    } // Eigen::
//    int m_columns = m.cols();
//    int m_rows = m.rows();
//    Eigen::ArrayXf mask = Eigen::ArrayXf::Zero(m_rows, m_columns);
//    Eigen::ArrayXf filtered_m_array = (m.array() > -0.8).select(mask, m);
//    filtered_m_array = (filtered_m_array < -0.6).select(mask, filtered_m_array);
//    grid_map::Matrix filtered_m = filtered_m_array.matrix();
//
//    float filtered_m_min_value = filtered_m.minCoeff();
//    float filtered_m_max_value = filtered_m.maxCoeff();
//
//    ROS_INFO("filtered_m min value: %s", std::to_string(filtered_m_min_value).c_str());
//    ROS_INFO("filtered_m max value: %s", std::to_string(filtered_m_max_value).c_str());

    ROS_INFO("matrix column: %s", std::to_string(m.cols()).c_str());
    cv::Mat map;
    grid_map::GridMapCvConverter::toImage<unsigned char,3>(gridMap, "elevation", CV_16UC3, map);
    cv::imwrite("/home/viplab/grid_map_output/grid_map_img_from_cv_converter.png", map);

    // eigen matrix to opencv image
    cv::Mat map_from_eigen;
    float min_value = gridMap.get("elevation").minCoeffOfFinites();
    float max_value = gridMap.get("elevation").maxCoeffOfFinites();
    ROS_INFO("matrix min value: %s", std::to_string(min_value).c_str());
    ROS_INFO("matrix max value: %s", std::to_string(max_value).c_str());
//    eigen2cv(m, map_from_eigen);
//    cv::imwrite("/home/viplab/map_from_eigen.png", map_from_eigen);

    // eigen matrix to local txt file
    std::ofstream file("/home/viplab/grid_map_output/grid_map_eigen.txt");
    if (file.is_open())
    {
        file << m << '\n';
    }


    // publish grid map

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(gridMap, msg);
    gridMapPub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_pcl_loader_node");
    ros::NodeHandle nh("~");
    gm::setVerbosityLevelToDebugIfFlagSet(nh);

//    grid_map::GridMapPclLoader gridMapPclLoader;


    ros::Rate loop_rate(10);
    sub = nh.subscribe(gm::getPointcloudTopic(nh), 1, pointcloud_callback);

    gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map_from_raw_pointcloud", 1, true);

/*
//    const std::string pathToCloud = gm::getPcdFilePath(nh);
    gridMapPclLoader.loadParameters(gm::getParameterPath());
//    gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

    gm::processPointcloud(&gridMapPclLoader, nh);

    grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
    gridMap.setFrameId(gm::getMapFrame(nh));

//    gm::saveGridMap(gridMap, nh, gm::getMapRosbagTopic(nh));

    // publish grid map

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(gridMap, msg);
    gridMapPub.publish(msg);
    */

    // run
    ros::spin();
    return EXIT_SUCCESS;
}
