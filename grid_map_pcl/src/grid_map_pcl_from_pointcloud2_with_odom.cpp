/*
 * grid_map_pcl_from_pointcloud2with_odom.cpp
 *
 *  Created on: Jan 27, 2022
 *      Author: Yibin Li
 */

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

#include "opencv2/core.hpp"
//#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"


#include "sensor_msgs/PointCloud2.h"


namespace gm = ::grid_map::grid_map_pcl;

ros::Publisher gridMapPub;
ros::Publisher localPointCloudPub;
ros::Subscriber sub;

grid_map::GridMapPclLoader gridMapPclLoader;
//ros::NodeHandle nh("~");

//void pointcloud_callback (sensor_msgs::PointCloud2ConstPtr& cloud_msg, nav_msgs::Odometry::ConstPtr& odom_msg){
void pointcloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg){
    // maybe move all process into this subscriber callback function
    bool save_grid_map_to_local = true;
    ROS_INFO("inside callback");
    ros::NodeHandle nh("~");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud_input);

    double odom_pose_position_x = odom_msg->pose.pose.position.x;
    double odom_pose_position_y = odom_msg->pose.pose.position.y;
    double odom_pose_position_z = odom_msg->pose.pose.position.z;

    ROS_INFO("pointcloud2 to pcl pointcloud conversion succeed!");
    ROS_INFO("frame id : %s", odom_msg->header.frame_id.c_str());
    ROS_INFO("odom time : %f", odom_msg->header.stamp.toSec());
    ROS_INFO("map time : %f", cloud_msg->header.stamp.toSec());
    ROS_INFO("position x: %f", odom_msg->pose.pose.position.x);
    ROS_INFO("position y: %f", odom_msg->pose.pose.position.y);
    ROS_INFO("position z: %f", odom_msg->pose.pose.position.z);

    // process point cloud and get local point cloud here. Should probably do it elsewhere
    pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(pcl_cloud_input);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-1.0 + odom_pose_position_x,1.0 + odom_pose_position_x);
    pass_x.filter(xf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xf_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-1.0 + odom_pose_position_x, 1.0 + odom_pose_position_y);
    pass_y.filter(yf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(yf_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-3.0 + odom_pose_position_z, 3.0 + odom_pose_position_z);
    pass_z.filter(zf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr zf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
    sensor_msgs::PointCloud2 LocalPointsMsg;
    pcl::toROSMsg(*zf_cloud_ptr, LocalPointsMsg);
    localPointCloudPub.publish(LocalPointsMsg);

//    gridMapPclLoader.setInputCloud(pcl_cloud_input); // global point cloud
    gridMapPclLoader.setInputCloud(zf_cloud_ptr); // local point cloud
    gridMapPclLoader.loadParameters(ros::package::getPath("grid_map_pcl") + "/config/parameters.yaml");
    gm::processPointcloud(&gridMapPclLoader, nh);

    grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();

////    print out layers in grid map
//    std::vector<std::string> layer_name = gridMap.getLayers();
//    for (auto &name : layer_name) {
//        ROS_INFO("layer name %s", name.c_str());
//    }

    std::string grid_map_col = std::to_string(gridMap.getSize()(1));
    std::string grid_map_row = std::to_string(gridMap.getSize()(0));

    ROS_INFO("grid map column size: %s", grid_map_col.c_str());
    ROS_INFO("grid map row size: %s", grid_map_row.c_str());

    grid_map::Matrix& m = gridMap.get("elevation");

    ROS_INFO("matrix column: %s", std::to_string(m.cols()).c_str());

    float min_value = gridMap.get("elevation").minCoeffOfFinites();
    float max_value = gridMap.get("elevation").maxCoeffOfFinites();
    ROS_INFO("matrix min value: %s", std::to_string(min_value).c_str());
    ROS_INFO("matrix max value: %s", std::to_string(max_value).c_str());
    ROS_INFO("=========================================================");
    ROS_INFO(" ");

//    bool success;
//    grid_map::Position pos = gridMap.getPosition();
//    grid_map::Position pos = grid_map::Position(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y); // use real odom info
//    ROS_INFO("Grid map position: %f %f\n", pos(0), pos(1));
//    ROS_INFO("Grid map length: %f %f\n", gridMap.getLength()(0), gridMap.getLength()(1));
//    grid_map::Length length(2.0, 2.0);
//    grid_map::GridMap localMap = gridMap.getSubmap(pos, length, success);
//    if (!success) {
//        ROS_WARN("getting local submap failed");
//    }

    if (save_grid_map_to_local) {
        cv::Mat map;
        grid_map::GridMapCvConverter::toImage<unsigned char,3>(gridMap, "elevation", CV_16UC3, map);
//        grid_map::GridMapCvConverter::toImage<unsigned char,3>(localMap, "elevation", CV_16UC3, map);
//        cv::imwrite("/home/viplab/grid_map_output/grid_map_img_from_cv_converter_" + odom_msg->header.stamp + ".png", map);
        cv::imwrite("/home/viplab/grid_map_output/grid_map_img_from_cv_converter.png", map);

////        eigen matrix to opencv image
//        cv::Mat map_from_eigen;
//
//        eigen2cv(m, map_from_eigen);
//        cv::imwrite("/home/viplab/map_from_eigen.png", map_from_eigen);
//
////        eigen matrix to local txt file
//        std::ofstream file("/home/viplab/grid_map_output/grid_map_eigen.txt");
//        if (file.is_open())
//        {
//            file << m << '\n';
//        }
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

    ros::Rate loop_rate(10);

    // time synchronizer
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc2_sub(nh, gm::getPointcloudTopic(nh), 100);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pc2_sub, odom_sub);

//    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(pc2_sub, odom_sub, 1);
    sync.registerCallback(boost::bind(&pointcloud_callback, _1, _2));

//    sub = nh.subscribe(gm::getPointcloudTopic(nh), 1, pointcloud_callback);

    gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map_from_raw_pointcloud", 1, true);
    localPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 100);

/*
//    const std::string pathToCloud = gm::getPcdFilePath(nh);
    gridMapPclLoader.loadParameters(gm::getParameterPath(nh));
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
