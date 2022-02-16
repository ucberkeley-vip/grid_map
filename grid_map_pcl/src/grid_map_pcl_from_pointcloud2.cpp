/*
 * grid_map_pcl_from_pointcloud2.cpp
 *
 *  Created on: Nov 7, 2021
 *      Author: Yibin Li
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/range_image/range_image.h>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"
#include <grid_map_pcl/PclLoaderParameters.hpp>

#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/core/eigen.hpp>

#include <tf/transform_listener.h>

#include "sensor_msgs/PointCloud2.h"


namespace gm = ::grid_map::grid_map_pcl;

ros::Publisher gridMapPub;
ros::Publisher gridImagePub;
ros::Subscriber sub;

grid_map::GridMapPclLoader gridMapPclLoader;

tf::TransformListener* tf_listener;

void pointcloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // maybe move all process into this subscriber callback function
    ROS_INFO("inside callback");
    ros::NodeHandle nh("~");

    ROS_INFO("Pointcloud dim: %d x %d", cloud_msg->height, cloud_msg->width);

    // sensor_msgs::PointCloud2 transformed;
    // bool success = pcl_ros::transformPointCloud("base_link", *cloud_msg, transformed, *tf_listener);
    // if (!success) {
    //     ROS_WARN("transformPointCloud to base_link failed");
    // }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud_input);
    ROS_INFO("pointcloud2 to pcl pointcloud conversion succeed!");

    gridMapPclLoader.setInputCloud(pcl_cloud_input);
    gm::processPointcloud(&gridMapPclLoader, nh);

    // grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
    
    grid_map::GridMap originalMap = gridMapPclLoader.getGridMap();
    grid_map::GridMap gridMap;
    grid_map::GridMapCvProcessing::changeResolution(originalMap, gridMap, 0.001);
    gridMap.setFrameId(gm::getMapFrame(nh));
    // gridMap.setFrameId(*cloud_msg.header.frame_id);

    // std::vector<std::string> layer_name = gridMap.getLayers();
    // for (auto &name : layer_name) {
    //     ROS_INFO("layer name %s", name.c_str());
    // }

    // std::string grid_map_col = std::to_string(gridMap.getSize()(1));
    // std::string grid_map_row = std::to_string(gridMap.getSize()(0));

    // ROS_INFO("grid map column size: %s", grid_map_col.c_str());
    // ROS_INFO("grid map row size: %s", grid_map_row.c_str());

    bool success;
    grid_map::Position pos = gridMap.getPosition();
    ROS_INFO("Grid map position: %f %f\n", pos(0), pos(1));
    ROS_INFO("Grid map length: %f %f\n", gridMap.getLength()(0), gridMap.getLength()(1));
    grid_map::Length length(2.0, 2.0);
    grid_map::GridMap localMap = gridMap.getSubmap(pos, length, success);
    if (!success) {
        ROS_WARN("getting local submap failed");
    }
    // localMap.setGeometry(gridMap.getLength());

    // grid_map::Index submapStartIndex(3, 5);
    // grid_map::Index submapBufferSize(12, 7);

    // for (grid_map::SubmapIterator iterator(gridMap, submapStartIndex, submapBufferSize);
    //     !iterator.isPastEnd(); ++iterator) {
    //     localMap.at("elevation", *iterator) = gridMap.at("elevation", *iterator);
    // }


    cv::Mat map;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(gridMap, "elevation", CV_8UC1, map);
    auto pathName = ros::package::getPath("grid_map_pcl");
    ROS_INFO("package name: %s", pathName.c_str());
    cv::imwrite(pathName + "/grid_image.png", map);
    
    // double min, max;
    // cv::minMaxLoc(raw, &min, &max);
    // ROS_INFO("image data: %f %f", min, max);

    // cv::imshow("Grid map", map);
    // cv::waitKey(1);

    // float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    // float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    // float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    // Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    // pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    // float noiseLevel=0.00;
    // float minRange = 0.0f;
    // int borderSize = 1;

    // pcl::RangeImage rangeImage;
    // rangeImage.createFromPointCloud(pcl_cloud_input, angularResolution, 
    // maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, 
    // noiseLevel, minRange, borderSize);

    sensor_msgs::Image ros_image;
    pcl::toROSMsg(*cloud_msg, ros_image);
    ROS_INFO("ros img dim: %d %d", ros_image.height, ros_image.width);
    gridImagePub.publish(ros_image);
    
    // publish grid map

    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(gridMap, msg);
    gridMapPub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_from_pointcloud2");
    ros::NodeHandle nh("~");
    gm::setVerbosityLevelToDebugIfFlagSet(nh);

    tf_listener = new tf::TransformListener();

    gridMapPclLoader.loadParameters(gm::getParameterPath(nh));

    ros::Rate loop_rate(10);
    sub = nh.subscribe(gm::getPointcloudTopic(nh), 1, pointcloud_callback);

    gridMapPub = nh.advertise<grid_map_msgs::GridMap>("/grid_map_from_raw_pointcloud", 1, true);
    gridImagePub = nh.advertise<sensor_msgs::Image>("/grid_map_image", 3, true);

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
