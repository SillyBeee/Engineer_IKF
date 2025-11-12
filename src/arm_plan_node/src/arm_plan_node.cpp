#include <arm_plan_node/arm_plan_node.hpp>

ArmPlanNode::ArmPlanNode(rclcpp::NodeOptions options) : Node("arm_plan_node", options) {
  RCLCPP_INFO(this->get_logger(), "ArmPlanNode has been started.");
  //参数初始化与发布订阅初始化
  this->planning_group_name_ = this->declare_parameter<std::string>("planning_group_name", "manipulator");
  this->end_effector_link_name_ = this->declare_parameter<std::string>("end_effector_link_name", "ee_link");
  this->debug_ = this->declare_parameter<bool>("debug", true);
  RCLCPP_INFO(this->get_logger(), "Planning Group: %s", this->planning_group_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", this->end_effector_link_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug mode: %s", this->debug_ ? "true" : "false");
  


  //Moveit组件初始化
  std::thread([this](){
    try{
      this->move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), this->planning_group_name_);
      this->planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
      move_group_->setPlanningTime(5.0);
      move_group_->setNumPlanningAttempts(10);
      move_group_->setMaxVelocityScalingFactor(0.9);
      move_group_->setMaxAccelerationScalingFactor(0.9);
      move_group_->setGoalPositionTolerance(0.001);   // 1mm位置容差
      move_group_->setGoalOrientationTolerance(0.01); // ~0.6度方向容差
      move_group_->setEndEffectorLink(this->end_effector_link_name_);
      RCLCPP_INFO(this->get_logger(), "MoveIt组件初始化完成");
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt组件初始化失败: %s", e.what());
    }
  }).detach();
}


int main(){
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<ArmPlanNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
