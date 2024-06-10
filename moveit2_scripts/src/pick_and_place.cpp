#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

// Logger for the node
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("pick_and_place_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  // Initialize the planning scene interface for collision checking
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Define a collision object for the table
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = move_group_arm.getPlanningFrame();
  table.id = "table";

  // Define a box for the table surface
  shape_msgs::msg::SolidPrimitive table_surface;
  table_surface.type = table_surface.BOX;
  table_surface.dimensions.resize(3);
  table_surface.dimensions[0] = 1;    // x dimension
  table_surface.dimensions[1] = 1.8;  // y dimension
  table_surface.dimensions[2] = 0.01; // z dimension, very thin

  // Define the pose of the table
  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.3;
  table_pose.position.y = 0.3;
  table_pose.position.z = -0.005; // Position it slightly below Z=0

  table.primitives.push_back(table_surface);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  // Define a collision object for the coffee maker
  moveit_msgs::msg::CollisionObject coffee_maker;
  coffee_maker.header.frame_id = move_group_arm.getPlanningFrame();
  coffee_maker.id = "coffee_maker";

  // Define a box for the coffee maker
  shape_msgs::msg::SolidPrimitive coffee_maker_surface;
  coffee_maker_surface.type = coffee_maker_surface.BOX;
  coffee_maker_surface.dimensions.resize(3);
  coffee_maker_surface.dimensions[0] = 0.8; // x dimension
  coffee_maker_surface.dimensions[1] = 0.2; // y dimension
  coffee_maker_surface.dimensions[2] = 0.8; // z dimension

  // Define the pose of the coffee maker
  geometry_msgs::msg::Pose coffee_maker_pose;
  coffee_maker_pose.orientation.w = 1.0;
  coffee_maker_pose.position.x = 0.30;
  coffee_maker_pose.position.y = 0.75;
  coffee_maker_pose.position.z = 0.4;

  coffee_maker.primitives.push_back(coffee_maker_surface);
  coffee_maker.primitive_poses.push_back(coffee_maker_pose);
  coffee_maker.operation = coffee_maker.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(table);
  collision_objects.push_back(coffee_maker);

  // Add the collision objects to the world
  planning_scene_interface.addCollisionObjects(collision_objects);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Get joint model groups for the arm and gripper
  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get current state
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Move the arm to the home position
  RCLCPP_INFO(LOGGER, "Moving arm to home position");

  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
  bool success_arm =
      (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to home position");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(arm_plan);

  // Move the arm to the pre-grasp position
  RCLCPP_INFO(LOGGER, "Moving arm to pre-grasp position");

  geometry_msgs::msg::Pose pre_grasp_pose;
  pre_grasp_pose.orientation.x = 0;
  pre_grasp_pose.orientation.y = -1;
  pre_grasp_pose.orientation.z = 0.0;
  pre_grasp_pose.orientation.w = 0.00;
  pre_grasp_pose.position.x = 0.30;
  pre_grasp_pose.position.y = 0.33;
  pre_grasp_pose.position.z = 0.33;
  move_group_arm.setPoseTarget(pre_grasp_pose);

  success_arm =
      (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to pre-grasp position");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(arm_plan);

  // Open the gripper
  RCLCPP_INFO(LOGGER, "Opening gripper");

  move_group_gripper.setNamedTarget("gripper_open");

  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
  bool success_gripper = (move_group_gripper.plan(gripper_plan) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(gripper_plan);

  // Approach the object
  RCLCPP_INFO(LOGGER, "Approaching object");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  pre_grasp_pose.position.z -= 0.04;
  approach_waypoints.push_back(pre_grasp_pose);
  pre_grasp_pose.position.z -= 0.04;
  approach_waypoints.push_back(pre_grasp_pose);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  if (fraction < 0.9) {
    RCLCPP_ERROR(LOGGER, "Failed to plan a sufficient approach path");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(trajectory_approach);

  // Close the gripper
  RCLCPP_INFO(LOGGER, "Closing gripper");

  move_group_gripper.setNamedTarget("gripper_close");

  success_gripper = (move_group_gripper.plan(gripper_plan) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to close gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(gripper_plan);

  // Retreat from the object
  RCLCPP_INFO(LOGGER, "Retreating from object");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  for (int i = 0; i < 8; ++i) {
    pre_grasp_pose.position.z += 0.01;
    retreat_waypoints.push_back(pre_grasp_pose);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  if (fraction < 0.9) {
    RCLCPP_ERROR(LOGGER, "Failed to plan a sufficient retreat path");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(trajectory_retreat);

  // Move the arm to the release position
  RCLCPP_INFO(LOGGER, "Moving arm to release position");

  current_state_arm = move_group_arm.getCurrentState(10);
  if (!current_state_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to get current state of the arm");
    rclcpp::shutdown();
    return -1;
  }

  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] += 2.3561944901923; // Shoulder Pan
  if (!move_group_arm.setJointValueTarget(joint_group_positions_arm)) {
    joint_group_positions_arm[0] -= 2.3561944901923;
    joint_group_positions_arm[0] -= 3.14159265358979323846;
    joint_group_positions_arm[0] -= 0.78539816339744831;
  }

  move_group_arm.setPlanningTime(20.0); // Increase the planning time

  success_arm =
      (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to release position");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(arm_plan);

  // Lower the end-effector
  RCLCPP_INFO(LOGGER, "Lowering the end-effector");

  std::vector<geometry_msgs::msg::Pose> lower_waypoints;
  geometry_msgs::msg::Pose current_pose = move_group_arm.getCurrentPose().pose;
  lower_waypoints.push_back(current_pose);

  current_pose.position.z -= 0.6;
  lower_waypoints.push_back(current_pose);

  moveit_msgs::msg::RobotTrajectory trajectory_lower;
  double fraction_lower = move_group_arm.computeCartesianPath(
      lower_waypoints, eef_step, jump_threshold, trajectory_lower);

  if (fraction_lower < 0.9) {
    RCLCPP_ERROR(LOGGER, "Failed to plan a sufficient lowering path");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(trajectory_lower);

  // Open the gripper to release the object
  RCLCPP_INFO(LOGGER, "Releasing object");

  move_group_gripper.setNamedTarget("gripper_open");

  success_gripper = (move_group_gripper.plan(gripper_plan) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(gripper_plan);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
