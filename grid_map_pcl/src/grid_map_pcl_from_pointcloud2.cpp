/*
 * grid_map_pcl_from_pointcloud2.cpp
 *
 *  Created on: Nov 7, 2021
 *      Author: Yibin Li
 */

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

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

    // change resolution to 0.02(m). This is not ideal, should change resolution in some param file.
//    grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
//    gridMap.setFrameId(gm::getMapFrame(nh));

    grid_map::GridMap originalMap = gridMapPclLoader.getGridMap();
    grid_map::GridMap gridMap;
    grid_map::GridMapCvProcessing::changeResolution(originalMap, gridMap, 0.02);
    gridMap.setFrameId(gm::getMapFrame(nh));

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
