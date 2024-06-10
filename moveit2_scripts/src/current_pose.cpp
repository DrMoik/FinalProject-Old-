#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_pose_demo");

void printCurrentPose(moveit::planning_interface::MoveGroupInterface &move_group) {
  auto current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(LOGGER, "Current Pose:");
  RCLCPP_INFO(LOGGER, "Position - x: %f, y: %f, z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  RCLCPP_INFO(LOGGER, "Orientation - x: %f, y: %f, z: %f, w: %f", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_pose_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  
  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);

  // Print the current pose
  printCurrentPose(move_group_arm);

  rclcpp::shutdown();
  return 0;
}
