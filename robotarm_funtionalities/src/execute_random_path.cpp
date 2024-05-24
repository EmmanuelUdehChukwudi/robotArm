#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("execute_trajectory_node");

  // Start a ROS spinning thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create MoveGroupInterface
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");

  // Set a random target
  move_group->setRandomTarget();

  // Get the target pose
  geometry_msgs::msg::Pose target_pose = move_group->getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Target pose: x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
              target_pose.position.x,
              target_pose.position.y,
              target_pose.position.z,
              target_pose.orientation.x,
              target_pose.orientation.y,
              target_pose.orientation.z,
              target_pose.orientation.w);

  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Planning %s", success ? "SUCCESSFUL" : "FAILED");

  // Execute the plan if successful
  if (success)
  {
    move_group->move();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
