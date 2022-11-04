#include "rclcpp/rclcpp.hpp"
#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>
#include <ArNetworking/ArClientRatioDrive.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <omron_ros2_msgs/srv/dock_request.hpp>
#include <omron_ros2_msgs/msg/omron.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "angles/angles.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

class statusPub: public rclcpp::Node
{
public:
  statusPub(ArClientBase *client, std::string name="Pose", std::string topic="/pose"); 
  void pose_cb(ArNetPacket *packet);
  void dock_stats_cb(ArNetPacket *packet);
  void simplePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void LocaliseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool requestDock(const std::shared_ptr<omron_ros2_msgs::srv::DockRequest::Request> req, std::shared_ptr<omron_ros2_msgs::srv::DockRequest::Response>  res);

  //Public robot stats
  int robotMode = 0;
  int robotStatus = 0;
  int dock_status = 0;

  geometry_msgs::msg::PoseStamped currentPose;

private:

  rclcpp::Publisher<omron_ros2_msgs::msg::Omron>::SharedPtr status_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initpose;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;



  ArClientBase *myClient;

  ArFunctor1C<statusPub, ArNetPacket *> myPoseCB;
  ArFunctor1C<statusPub, ArNetPacket *> myStatusCB;
  ArFunctor1C<statusPub, ArNetPacket *> dockedStatusCB;

  int seq = 0;


  rclcpp::Service<omron_ros2_msgs::srv::DockRequest>::SharedPtr service;

  void cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelWD();


  //Cmd Vel variables
  rclcpp::TimerBase::SharedPtr timer_;
  uint8_t velCount;
  uint8_t prevVelCount;
  bool vel_valid;

};


class laserPub: public rclcpp::Node
{
public:
  laserPub(ArClientBase *client, std::string name="Laser_1Current", std::string topic="/laser"); 
  void laser_cb(ArNetPacket *packet);

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laser_pub;
  ArClientBase *myClient;
  ArFunctor1C<laserPub, ArNetPacket *> myLaserCB;

  int seq = 0;

};


class mapPub: public rclcpp::Node
{
public:
  mapPub(ArClientBase *client); 


  geometry_msgs::msg::PoseStamped currentPose;

protected:

  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;


  ArClientBase *myClient;

  ArFunctor1C<mapPub, ArNetPacket *> getMapNameCB;
  ArFunctor1C<mapPub, ArNetPacket *> getMapCB;

  ArMap arMap;
  ArClientBase client;
  ArTime start;


  void handleGetMapName(ArNetPacket *packet);
  void handleGetMap(ArNetPacket *packet);

  int mapped = 0;

  nav_msgs::msg::MapMetaData    meta_data_message_;
  nav_msgs::msg::OccupancyGrid  map_resp_;


};