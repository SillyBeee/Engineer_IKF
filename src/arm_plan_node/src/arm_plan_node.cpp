#include <arm_plan_node/arm_plan_node.hpp>
#include <boost/token_functions.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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
  this->sub_target_end_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_end_pose", 10,
    std::bind(&ArmPlanNode::TargetEndPoseCallback, this, std::placeholders::_1)
  );
  this->sub_target_joint_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_joint_pose", 10,
    std::bind(&ArmPlanNode::TargetJointPoseCallback, this, std::placeholders::_1)
  );
  this->sub_joint_state_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/communicate/miniarm_pos", 10,
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
  //获取当前关节状态
  moveit::core::RobotStatePtr current_robot_state =
        move_group_->getCurrentState();
  if (current_robot_state) {
        std::stringstream ss;
        current_robot_state->printStatePositions(ss);
        if (debug_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "当前机械臂状态 (关节位置): %s",
                ss.str().c_str()
            );
        }
  } else {
      RCLCPP_WARN(this->get_logger(), "无法获取当前机械臂状态以记录初始状态");
  }
  if (debug_) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "目标位姿: %f, %f, %f",
          target_pose.pose.position.x,
          target_pose.pose.position.y,
          target_pose.pose.position.z
      );
  }
  RCLCPP_INFO(this->get_logger(), "规划到目标位姿");

  //设置目标位姿并进行规划
  move_group_->setPoseTarget(target_pose);
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(this->get_logger(), "运动规划成功");
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = current_plan_.start_state;
    display_trajectory.trajectory.push_back(current_plan_.trajectory);
    if (debug_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "生成的轨迹路径点数量: %zu",
                display_trajectory.trajectory.size()
            );
    } 
    pub_display_trajectory_->publish(display_trajectory);
  } else {
    RCLCPP_WARN(this->get_logger(), "运动规划失败");
  }
  return success;
}

bool ArmPlanNode::PlanToPose(const std::vector<double>& target_joint_pose){

  if(target_joint_pose.size() != this->joints_num_){
    RCLCPP_WARN(this->get_logger(), "目标关节姿态数量与预期不符，规划失败");
    return false;
  }

   //获取当前关节状态
  moveit::core::RobotStatePtr current_robot_state =
        move_group_->getCurrentState();
  if (current_robot_state) {
        std::stringstream ss;
        current_robot_state->printStatePositions(ss);
        if (debug_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "当前机械臂状态 (关节位置): %s",
                ss.str().c_str()
            );
        }
  } else {
      RCLCPP_WARN(this->get_logger(), "无法获取当前机械臂状态以记录初始状态");
  }
  if (debug_) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "目标关节姿态: [%f, %f, %f, %f, %f, %f]",
          target_joint_pose[0],
          target_joint_pose[1],
          target_joint_pose[2],
          target_joint_pose[3],
          target_joint_pose[4],
          target_joint_pose[5]
      );
  }
  RCLCPP_INFO(this->get_logger(), "规划到目标关节角度");

  //设置目标关节姿态并进行规划
  move_group_->setJointValueTarget(target_joint_pose);
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(this->get_logger(), "运动规划成功");
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = current_plan_.start_state;
    display_trajectory.trajectory.push_back(current_plan_.trajectory);
    if (debug_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "生成的轨迹路径点数量: %zu",
                display_trajectory.trajectory.size()
            );
    } 
    pub_display_trajectory_->publish(display_trajectory);
  } else {
    RCLCPP_WARN(this->get_logger(), "运动规划失败");
  }
  return success;
}

bool ArmPlanNode::SolveIK(
    const geometry_msgs::msg::Pose& target_pose,
    std::vector<double>& joint_pose)
{
  if (!move_group_) {
    RCLCPP_WARN(this->get_logger(), "MoveGroup 尚未初始化完成，无法进行 IK 求解");
    return false;
  }

  auto robot_model = move_group_->getRobotModel();
  const auto* jmg = robot_model->getJointModelGroup(planning_group_name_);
  if (!jmg) {
    RCLCPP_ERROR(this->get_logger(), "关节组 %s 不存在", planning_group_name_.c_str());
    return false;
  }

  moveit::core::RobotState state(robot_model);
  state = *move_group_->getCurrentState();

  // 当前版本的正确调用方式：group + pose + timeout
  bool ok = state.setFromIK(
      jmg,
      target_pose,
      0.1  // timeout 秒
      // 其他参数用默认值即可
  );

  if (!ok) {
    RCLCPP_WARN(this->get_logger(), "IK 求解失败");
    return false;
  }

  joint_pose.clear();
  state.copyJointGroupPositions(jmg, joint_pose);
  return true;
}

bool ArmPlanNode::ExecutePlan(){
  RCLCPP_INFO(this->get_logger(), "执行运动计划");

  if(debug_){
    RCLCPP_INFO(this->get_logger(),"在虚拟环境中运行");
    bool success =
    (move_group_->execute(current_plan_)
      == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "运动执行成功");
    } else {
        RCLCPP_ERROR(this->get_logger(), "运动执行失败");
    }

    return success;
  }
  else{
    RCLCPP_INFO(this->get_logger(),"在真实环境中运行");
    return true;
  }
}


void ArmPlanNode::TargetEndPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() != 7) {
    RCLCPP_WARN(this->get_logger(), "接收到的目标末端位姿数量与预期不符，忽略该消息");
    return;
  }
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link"; // 根据具体机械臂的基座链接名称进行修改
  target_pose.pose.position.x = msg->data[0];
  target_pose.pose.position.y = msg->data[1];
  target_pose.pose.position.z = msg->data[2];
  target_pose.pose.orientation.x = msg->data[3];
  target_pose.pose.orientation.y = msg->data[4];
  target_pose.pose.orientation.z = msg->data[5];
  target_pose.pose.orientation.w = msg->data[6];

  bool plan_success = this->PlanToPose(target_pose);
  if (!plan_success) {
    RCLCPP_WARN(this->get_logger(), "根据接收到的目标末端位姿规划失败");
  }
}

void ArmPlanNode::TargetJointPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() != this->joints_num_) {
    RCLCPP_WARN(this->get_logger(), "接收到的目标关节姿态数量与预期不符，忽略该消息");
    RCLCPP_WARN(this->get_logger(), "Expected %d, but got %zu", this->joints_num_, msg->data.size());
    return;
  }
  std::vector<double> target_joint_pose(msg->data.begin(), msg->data.end());
  bool plan_success = this->PlanToPose(target_joint_pose);
  if (!plan_success) {
    RCLCPP_WARN(this->get_logger(), "根据接收到的目标关节姿态规划失败");
  }
  ExecutePlan();
} 

void ArmPlanNode::JointStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if(msg->data.size()!= this->joints_num_){
    RCLCPP_WARN(this->get_logger(), "接收到的关节状态数量与预期不符，忽略该消息");
    RCLCPP_WARN(this->get_logger(), "Expected %d, but got %zu", this->joints_num_, msg->data.size());
    return;
  }
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.name.resize(this->joints_num_);
  joint_state_msg.position.resize(this->joints_num_);
  //TODO: 根据具体机械臂的关节命名规则进行修改
  
  
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // 这一行保留：允许用 YAML 覆盖 / 声明参数
  // options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<ArmPlanNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
