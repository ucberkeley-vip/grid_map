/*
 * grid_map_pcl_from_pointcloud2with_odom.cpp
 *
 *  Created on: Jan 27, 2022
 *      Author: Yibin Li
 */

#include <iostream>
#include <string>
#include <cmath>

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
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui.hpp"

#include "edlines.h"
#include "edlines.cpp"

#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"


namespace gm = ::grid_map::grid_map_pcl;

ros::Publisher gridMapPub;
ros::Publisher localPointCloudPub;
ros::Publisher joistDistancePub;
ros::Subscriber sub;

grid_map::GridMapPclLoader gridMapPclLoader;
//ros::NodeHandle nh("~");

// process point cloud and get local point cloud here. Should probably do it elsewhere
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_input,
                                                      double x_limit_min, double x_limit_max,
                                                      double y_limit_min, double y_limit_max,
                                                      double z_limit_min, double z_limit_max) {

    pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(pointcloud_input);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_limit_min, x_limit_max);
    pass_x.filter(xf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xf_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_limit_min, y_limit_max);
    pass_y.filter(yf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(yf_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_limit_min, z_limit_max);
    pass_z.filter(zf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr zf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
    return zf_cloud_ptr;
}

// return the distance of closest horizontal line to the image bottom, also draw lines on image
float get_edline_detection(cv::Mat& img) {
    cv::Mat gray_image;
    cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
    int W = img.cols;
    int H = img.rows;
    int image_size = W * H;
    unsigned char* input = new unsigned char[image_size];
    memcpy(input, gray_image.data, image_size);
    std::vector<line_float_t> Lines;
    boundingbox_t Bbox = { 0,0,W,H };
    float scalex =1;
    float scaley =1;
    int Flag = 0;
    Flag = EdgeDrawingLineDetector(input, W, H, scalex, scaley, Bbox, Lines);
    ROS_INFO("line detection status: %d", Flag); // 0 means ok
    std_msgs::Float32 min_joist_distance_msg;
    float min_joist_distance = 999;
    for (int i = 0; i < Lines.size(); i++)
    {
        // find the horizontal line that closes to the bottom
        // horizontal line has less than 0.2 rad (11.45 degree) of tilt

        if ((atan(abs(Lines[i].starty - Lines[i].endy) / abs(Lines[i].startx - Lines[i].endx))) < 0.2) {
            float midy = (Lines[i].starty + Lines[i].endy) / 2;
            ROS_INFO("midy: %f", midy);
            min_joist_distance = std::min(min_joist_distance, H - abs(midy)); // note that the top row is 0 and the bottom row is H
            min_joist_distance_msg.data = min_joist_distance;

            // only shows the potential joist
            line(img, cv::Point(Lines[i].startx, Lines[i].starty), cv::Point(Lines[i].endx, Lines[i].endy), cv::Scalar(0, 255, 0), 2);
        }

//        line(img, cv::Point(Lines[i].startx, Lines[i].starty), cv::Point(Lines[i].endx, Lines[i].endy), cv::Scalar(0, 0, 255), 2);
    }
    joistDistancePub.publish(min_joist_distance_msg);
}

// function to calculate the average height near "potential joist"
float get_average_grid_height(grid_map::GridMap grid_map, grid_map::Position& top_left, grid_map::Position& bottom_right) {
    int grid_map_col = grid_map.getSize()(1);
    int grid_map_row = grid_map.getSize()(0);
    int x1 = round(top_left.x());
    int y1 = round(top_left.y());
    int x2 = round(bottom_right.x());
    int y2 = round(bottom_right.y());
    float height_sum = 0;
    int valid_cell_num = 0;
    for (int i = x1-2; i < x2+3; i++) {
        for (int j = y1-2; i < y2+3; i++) {
            if (i > -1 && i < grid_map_col && j > -1 && j < grid_map_row) {
                height_sum += grid_map.atPosition("elevation", grid_map::Position(i, j));
                valid_cell_num++;
            }
        }
    }
    return height_sum / valid_cell_num;
}

void pointcloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg){
    // maybe move all process into this subscriber callback function
    bool save_grid_map_to_local = true;
    bool save_grid_map_matrix_to_local = false;
    bool line_detection = true;

    ROS_INFO("inside callback");
    ros::NodeHandle nh("~");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*pcl_cloud_input);

    double odom_pose_position_x = odom_msg->pose.pose.position.x;
    double odom_pose_position_y = odom_msg->pose.pose.position.y;
    double odom_pose_position_z = odom_msg->pose.pose.position.z;
    double odom_time = odom_msg->header.stamp.toSec();
    double map_time = cloud_msg->header.stamp.toSec();

    ROS_INFO("pointcloud2 to pcl pointcloud conversion succeed!");
    ROS_INFO("odom time : %f", odom_time);
    ROS_INFO("map time : %f", map_time);
    ROS_INFO("position x: %f", odom_pose_position_x);
    ROS_INFO("position y: %f", odom_pose_position_y);
    ROS_INFO("position z: %f", odom_pose_position_z);

    // the output image doesn't always have the same size as input pointcloud, therefore it is hard to localize where
    // robot is in the image. To make -0.6 and 0.6 for y we know the robot is always in the middle.
    pcl::PointCloud<pcl::PointXYZ>::Ptr zf_cloud_ptr = pointcloud_filter(pcl_cloud_input,
                                                                         0 + odom_pose_position_x, // x lim min
                                                                         1 + odom_pose_position_x, // x lim max
                                                                         -0.6 + odom_pose_position_y, // y lim min
                                                                         0.6 + odom_pose_position_y, // y lim max
                                                                         -0.5 + odom_pose_position_z, // z lim min
                                                                         -0.2 + odom_pose_position_z); // z lim max

    sensor_msgs::PointCloud2 LocalPointsMsg;
    pcl::toROSMsg(*zf_cloud_ptr, LocalPointsMsg);
    localPointCloudPub.publish(LocalPointsMsg);

//    gridMapPclLoader.setInputCloud(pcl_cloud_input); // global point cloud
    gridMapPclLoader.setInputCloud(zf_cloud_ptr); // local point cloud that used for now
//    // load parameters from local file doesn't work for now
//    gridMapPclLoader.loadParameters(ros::package::getPath("grid_map_pcl") + "/config/grid_map_odom_parameters.yaml");
    gm::processPointcloud(&gridMapPclLoader, nh);

    grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();

////    change map resolution (default is 0.01)
//    grid_map::GridMap originalMap = gridMapPclLoader.getGridMap();
//    grid_map::GridMap gridMap;
//    grid_map::GridMapCvProcessing::changeResolution(originalMap, gridMap, 0.005);

    gridMap.setFrameId(gm::getMapFrame(nh));

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

    float min_value = gridMap.get("elevation").minCoeffOfFinites();
    float max_value = gridMap.get("elevation").maxCoeffOfFinites();
    ROS_INFO("matrix min value: %s", std::to_string(min_value).c_str());
    ROS_INFO("matrix max value: %s", std::to_string(max_value).c_str());

    cv::Mat map;
    grid_map::GridMapCvConverter::toImage<unsigned char, 3>(gridMap, "elevation", CV_16UC3, map);
    std::string img_name = "/home/viplab/grid_map_output/grid_map_img.png";
    std::string img_with_line_detection_name = "/home/viplab/grid_map_output/grid_map_img_with_line_detection.png";

    if (save_grid_map_to_local) {
        img_name = "/home/viplab/grid_map_output/grid_map_img_" + std::to_string(odom_time) + ".png";

        std::string img_with_L515_name =
                "/home/viplab/grid_map_output/grid_map_img_with_L515_" + std::to_string(odom_time) + ".png";
        img_with_line_detection_name =
                "/home/viplab/grid_map_output/grid_map_img_with_line_detection_" + std::to_string(odom_time) + ".png";

//        // draw L515 position as filled circle
//        cv::Mat img = cv::imread(img_name, cv::IMREAD_COLOR);
//        cv::Point centerL515(floor(img.cols/2), img.rows-10);
//        cv::Scalar color(0,100,0);
//        cv::circle(img, centerL515, 10, color, cv::FILLED);
//        cv::imwrite(img_with_L515_name, img);
    }

    cv::imwrite(img_name, map);

    if (line_detection) {
        cv::Mat map_img = cv::imread(img_name);
//        get_edline_detection(map);
//        cv::imwrite(img_with_line_detection_name, map);

        get_edline_detection(map_img);
//        cv::rectangle(map_img, cv::Point(0, 0), cv::Point(map_img.cols-2, map_img.rows-2), cv::Scalar(0, 255, 0));
        cv::imwrite(img_with_line_detection_name, map_img);
    }

    if (save_grid_map_matrix_to_local) {
//        eigen matrix to local txt file
        std::ofstream file("/home/viplab/grid_map_output/grid_map_eigen.txt");
        if (file.is_open())
        {
            file << m << '\n';
        }
    }

    // publish grid map
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(gridMap, msg);
    gridMapPub.publish(msg);

    ROS_INFO("=========================================================");
    ROS_INFO(" ");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_pcl_loader_node");
    ros::NodeHandle nh("~");
    gm::setVerbosityLevelToDebugIfFlagSet(nh);

    ros::Rate loop_rate(10);

    // time synchronizer
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc2_sub(nh, gm::getPointcloudTopic(nh), 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pc2_sub, odom_sub);

    sync.registerCallback(boost::bind(&pointcloud_callback, _1, _2));

    gridMapPub = nh.advertise<grid_map_msgs::GridMap>("/grid_map_from_raw_pointcloud", 1, true);
    localPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 100);
    joistDistancePub = nh.advertise<std_msgs::Float32>("/joist_distance", 100);

    // run
    ros::spin();
    return EXIT_SUCCESS;
}
