#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

class FastLioKeyframeTap {
public:
  FastLioKeyframeTap(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) 
  {
    // Params (you can change later)
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");

    // Subs
    odom_sub_  = nh_.subscribe(odom_topic_, 100, &FastLioKeyframeTap::odomCb, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 10,  &FastLioKeyframeTap::cloudCb, this);

    ROS_INFO_STREAM("Subscribing to " << odom_topic_ << " and " << cloud_topic_);
  }

private:
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    // TODO: store latest pose, compute deltas vs last keyframe pose
    latest_odom_ = *msg;  // copy is fine for now
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // TODO: keep latest cloud; when a keyframe trigger fires, save this cloud with the pose
    latest_cloud_ = *msg;
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, cloud_sub_;
  std::string odom_topic_, cloud_topic_;

  nav_msgs::Odometry latest_odom_;
  sensor_msgs::PointCloud2 latest_cloud_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_keyframe_tap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  FastLioKeyframeTap node(nh, pnh);
  ros::spin();
  return 0;
}
