#include <waypoint_tool.hpp>

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace waypoint_rviz_plugin
{
WaypointTool::WaypointTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "/goal_pose", "The topic on which to publish navigation goal poses.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

WaypointTool::~WaypointTool() = default;

void WaypointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("GoalPose");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = raw_node->template create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5 ,std::bind(&WaypointTool::odomHandler,this,std::placeholders::_1));
  
  pub_ = raw_node->template create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", qos_profile_);
  pub_joy_ = raw_node->template create_publisher<sensor_msgs::msg::Joy>("/joy", qos_profile_);
  clock_ = raw_node->get_clock();
}

void WaypointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = clock_->now();
  joy.header.frame_id = "goal_pose_tool";
  pub_joy_->publish(joy);

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = joy.header.stamp;
  goal_pose.pose.position.x = x;
  goal_pose.pose.position.y = y;
  goal_pose.pose.position.z = vehicle_z;
  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, theta);
  goal_pose.pose.orientation = tf2::toMsg(yaw_quat);

  pub_->publish(goal_pose);
  usleep(10000);
  pub_->publish(goal_pose);
}
}

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::WaypointTool, rviz_common::Tool)
