#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_node");
  
  // Start a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Allow the node to start and the move group interface to initialize
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Create the MoveGroup interface for the arm
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a publisher for displaying planned paths
  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/move_group/display_planned_path", 10);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  // Log the reference frame names
  RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Define the target pose
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.15;
  target_pose1.position.y = 0.15;
  target_pose1.position.z = 0.15;
  move_group.setPoseTarget(target_pose1);

  // Plan the motion to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto success = move_group.plan(my_plan);

  // Log the planning result
  if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Plan successful");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Plan failed");
  }

  // Execute the planned motion
  move_group.move();

  // Shut down the ROS 2 node
  rclcpp::shutdown();
  return 0;
}
