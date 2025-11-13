#include <arm_plan_node/arm_plan_node.hpp>

ArmPlanNode::ArmPlanNode(rclcpp::NodeOptions options) : Node("arm_plan_node", options) {
  RCLCPP_INFO(this->get_logger(), "ArmPlanNode has been started.");
  //参数初始化与发布订阅初始化
  this->planning_group_name_ = this->declare_parameter<std::string>("planning_group_name", "manipulator");
  this->end_effector_link_name_ = this->declare_parameter<std::string>("end_effector_link_name", "ee_link");
  this->debug_ = this->declare_parameter<bool>("debug", true);
  this->joints_num_ = this->declare_parameter<uint8_t>("joints_num", 6);
  RCLCPP_INFO(this->get_logger(), "Planning Group: %s", this->planning_group_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", this->end_effector_link_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug mode: %s", this->debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Joints Number: %d", this->joints_num_);

  //发布订阅初始化
  this->sub_target_end_pose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "target_end_pose", 10,
    std::bind(&ArmPlanNode::TargetEndPoseCallback, this, std::placeholders::_1)
  );
  this->sub_target_joint_pose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "target_joint_pose", 10,
    std::bind(&ArmPlanNode::TargetJointPoseCallback, this, std::placeholders::_1)
  );
  this->sub_joint_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "joint_states", 10,
    std::bind(&ArmPlanNode::JointStateCallback, this, std::placeholders::_1)
  );
  this->pub_display_trajectory_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "display_planned_path", 10
  );
  this->pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10
  );


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

bool ArmPlanNode::PlanToPose(const geometry_msgs::msg::PoseStamped& target_pose) {
  move_group_->setPoseTarget(target_pose);
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(this->get_logger(), "运动规划成功");
  } else {
    RCLCPP_WARN(this->get_logger(), "运动规划失败");
  }
  return success;
}



void ArmPlanNode::TargetEndPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

}

void ArmPlanNode::TargetJointPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

}

void ArmPlanNode::JointStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if(msg->data.size()!= this->joints_num_){
    RCLCPP_WARN(this->get_logger(), "接收到的关节状态数量与预期不符，忽略该消息");
    return;
  }
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  
  
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
