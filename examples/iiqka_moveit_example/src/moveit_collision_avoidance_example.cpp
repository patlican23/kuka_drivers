#include <math.h>
#include <memory>
#include "iiqka_moveit_example/moveit_example.hpp"

// Function to add a custom collision box
void addCustomCollisionBox(std::shared_ptr<MoveitExample> example_node)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = example_node->moveGroupInterface()->getPlanningFrame();
  collision_object.id = "custom_collision_box";

  // Define the dimensions of the box
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.08;
  primitive.dimensions[primitive.BOX_Y] = 0.35;
  primitive.dimensions[primitive.BOX_Z] = 0.16;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.3;
  box_pose.position.y = -0.09;
  box_pose.position.z = 0.08;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  example_node->AddObject(collision_object);
}

void setStartStateToCommanded(std::shared_ptr<MoveitExample> example_node, const std::vector<double>& commanded_pos)
{
  moveit_msgs::msg::RobotState start_state;
  start_state.joint_state.name = example_node->moveGroupInterface()->getJointNames();
  start_state.joint_state.position = commanded_pos;
  example_node->moveGroupInterface()->setStartState(start_state);
}

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const example_node = std::make_shared<MoveitExample>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(example_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  example_node->initialize();
  example_node->addBreakPoint();

  // // Add robot platform
  // example_node->addRobotPlatform();

  // Move to initial position closer to the collision object
  std::vector<double> commanded_pos = {0.367, -2.059, -1.005, 2.922, 1.380, -2.761};
  auto init_trajectory = example_node->planToPosition(commanded_pos);
  if (init_trajectory != nullptr)
  {
    example_node->moveGroupInterface()->execute(*init_trajectory);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("moveit_example"), "Failed to plan initial trajectory");
    return 1;
  }

  // Synchronize the planning scene
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Add custom collision object
  addCustomCollisionBox(example_node);
  example_node->addBreakPoint();

  // Set the start state to the commanded position before planning
  setStartStateToCommanded(example_node, commanded_pos);

  // Define the target pose for the sample-based planner
  Eigen::Isometry3d target_pose = Eigen::Isometry3d(Eigen::Translation3d(0.4, 0.0, 0.55) * Eigen::Quaterniond::Identity());

  // Plan to the target pose using a sample-based planner (planToPoint)
  RCLCPP_INFO(rclcpp::get_logger("moveit_example"), "Planning to target pose");
  auto planned_trajectory = example_node->planToPoint(target_pose, "RRTConnectkConfigDefault");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  else
  {
    example_node->drawTitle("Failed to plan path to target pose");
    RCLCPP_ERROR(rclcpp::get_logger("moveit_example"), "Failed to plan path to target pose");
    return 1;
  }

  // Set the start state to the commanded position before planning
  setStartStateToCommanded(example_node, commanded_pos);

  // Plan to return to the initial position using the sample-based planner (planToPoint)
  RCLCPP_INFO(rclcpp::get_logger("moveit_example"), "Planning to initial pose");
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d(Eigen::Translation3d(0.1, 0.0, 0.8) * Eigen::Quaterniond::Identity());
  planned_trajectory = example_node->planToPoint(initial_pose, "RRTConnectkConfigDefault");
  if (planned_trajectory != nullptr)
  {
    example_node->drawTrajectory(*planned_trajectory);
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }
  else
  {
    example_node->drawTitle("Failed to plan path to initial position");
    RCLCPP_ERROR(rclcpp::get_logger("moveit_example"), "Failed to plan path to initial position");
    return 1;
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
