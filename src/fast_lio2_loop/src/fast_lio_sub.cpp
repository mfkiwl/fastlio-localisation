#include "fast_lio_sub.hpp"

FastLioPoll::FastLioPoll(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) 
  {
    // Params (you can change later)
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");

    // Subs
    odom_sub_  = nh_.subscribe(odom_topic_, 100, &FastLioPoll::odomCb, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 10,  &FastLioPoll::cloudCb, this);

    ROS_INFO_STREAM("Subscribing to " << odom_topic_ << " and " << cloud_topic_);
  }


void FastLioPoll::odomCb (const nav_msgs::Odometry::ConstPtr& msg) {
  // TODO: store latest pose, compute deltas vs last keyframe pose
  latest_odom_ = *msg;  // copy is fine for now
}

void FastLioPoll::cloudCb (const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // TODO: keep latest cloud; when a keyframe trigger fires, save this cloud with the pose
  latest_cloud_ = *msg;
}

