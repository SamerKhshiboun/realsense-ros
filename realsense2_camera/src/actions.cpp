#include "../include/base_realsense_node.h"
using namespace realsense2_camera;
using namespace rs2;

rclcpp_action::GoalResponse BaseRealSenseNode::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const TriggeredCalibration::Goal> goal)
{
  ROS_INFO("handle_goal");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BaseRealSenseNode::handle_cancel(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle)
{
    ROS_INFO("handle_cancel");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BaseRealSenseNode::handle_accepted(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle)
{
  using namespace std::placeholders;
  ROS_INFO("handle_accepted");
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&BaseRealSenseNode::execute, this, _1), goal_handle}.detach();
}

void BaseRealSenseNode::execute(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle)
{
  ROS_INFO("Executing goal");
}
