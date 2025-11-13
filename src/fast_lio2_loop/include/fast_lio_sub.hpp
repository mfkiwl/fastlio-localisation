#pragma once 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>



class FastLioPoll {
    public:
    FastLioPoll(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) ;

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber odom_sub_, cloud_sub_;
    std::string odom_topic_, cloud_topic_;

    nav_msgs::Odometry latest_odom_;
    sensor_msgs::PointCloud2 latest_cloud_;
};

