#include <arm_plan_node/arm_plan_node.hpp>
#include <boost/token_functions.hpp>
#include <memory>
#include <moveit_servo/utils/common.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ArmPlanNode::ArmPlanNode(rclcpp::NodeOptions options)
    : Node("arm_plan_node", options) {
  RCLCPP_INFO(this->get_logger(), "ArmPlanNode has been started.");
  // 参数初始化与发布订阅初始化
  this->get_parameter_or<std::string>(
      "planning_group_name", this->planning_group_name_, "mini_arm_group");
  this->get_parameter_or<std::string>("end_effector_link_name",
                                      this->end_effector_link_name_, "cc");
  this->get_parameter_or<bool>("debug", this->debug_, true);
  this->get_parameter_or<uint8_t>("joints_num", this->joints_num_, 6);
  RCLCPP_INFO(this->get_logger(), "Planning Group: %s",
              this->planning_group_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "End Effector Link: %s",
              this->end_effector_link_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug mode: %s",
              this->debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Joints Number: %d", this->joints_num_);

  // this->declare_parameter<std::string>("moveit_servo.move_group_name",
  //                                      this->planning_group_name_);
  // this->declare_parameter<std::string>("moveit_servo.planning_frame",
  //                                      "base_link");
  // this->declare_parameter<std::string>("moveit_servo.command_in_type",
  //                                      "unitless");

  // 为耗时的操作创建一个独立的回调组
  this->callback_group_time_consuming_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 将订阅选项单独拿出来，以便设置回调组
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group = this->callback_group_time_consuming_;

  // 发布订阅初始化
  this->sub_target_end_pose_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/target_end_pose", 10,
          std::bind(&ArmPlanNode::TargetEndPoseCallback, this,
                    std::placeholders::_1),
          subscription_options);
  this->sub_target_joint_pose_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/target_joint_pose", 10,
          std::bind(&ArmPlanNode::TargetJointPoseCallback, this,
                    std::placeholders::_1),
          subscription_options);
  this->sub_joint_state_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/communicate/miniarm_pos", 10,
          std::bind(&ArmPlanNode::JointStateCallback, this,
                    std::placeholders::_1));
  this->sub_joy_data_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/joy_data", 10,
          std::bind(&ArmPlanNode::JoyDataCallback, this, std::placeholders::_1),
          subscription_options);
  this->pub_display_trajectory_ =
      this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
          "display_planned_path", 10);
  this->pub_fake_controller_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "/mini_arm_controller/joint_trajectory", 10);
  this->pub_joint_state_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  this->pub_target_joint_pose_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/miniarm/joint_position", 10);
  this->pub_target_joint_vel_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "/miniarm/joint_velocity", 10);

  // Moveit组件初始化
  std::thread([this]() {
    try {
      this->move_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              this->shared_from_this(), this->planning_group_name_);
      this->planning_scene_interface_ = std::make_shared<
          moveit::planning_interface::PlanningSceneInterface>();
      move_group_->setPlanningTime(5.0);
      move_group_->setNumPlanningAttempts(10);
      move_group_->setMaxVelocityScalingFactor(0.9);
      move_group_->setMaxAccelerationScalingFactor(0.9);
      move_group_->setGoalPositionTolerance(0.001);   // 1mm位置容差
      move_group_->setGoalOrientationTolerance(0.01); // ~0.6度方向容差
      move_group_->setEndEffectorLink(this->end_effector_link_name_);
      RCLCPP_INFO(this->get_logger(), "MoveIt组件初始化完成");

      // Moveit Servo初始化
      const std::string param_namespace = "moveit_servo";
      auto node_ptr = this->shared_from_this();
      this->servo_param_listener_ =
          std::make_shared<servo::ParamListener>(node_ptr, param_namespace);
      this->servo_params_ = this->servo_param_listener_->get_params();
      this->planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(
          node_ptr, this->servo_params_);
      this->servo_ = std::make_shared<moveit_servo::Servo>(
          node_ptr, this->servo_param_listener_, this->planning_scene_monitor_);
      RCLCPP_INFO(this->get_logger(), "MoveIt Servo 初始化完成");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt 组件初始化失败: %s", e.what());
    }
  }).detach();
}

void ArmPlanNode::ChangeCtlType(ArmPlanNode::CtlType ctl_type) {
  this->ctl_type_ = ctl_type;
}

bool ArmPlanNode::PlanToPose(
    const geometry_msgs::msg::PoseStamped &target_pose) {
  // 获取当前关节状态
  moveit::core::RobotStatePtr current_robot_state =
      move_group_->getCurrentState();
  if (current_robot_state) {
    std::stringstream ss;
    current_robot_state->printStatePositions(ss);
    if (debug_) {
      RCLCPP_DEBUG(this->get_logger(), "当前机械臂状态 (关节位置): %s",
                   ss.str().c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "无法获取当前机械臂状态以记录初始状态");
  }
  if (debug_) {
    RCLCPP_DEBUG(this->get_logger(), "目标位姿: %f, %f, %f",
                 target_pose.pose.position.x, target_pose.pose.position.y,
                 target_pose.pose.position.z);
  }
  RCLCPP_INFO(this->get_logger(), "规划到目标位姿");

  // 设置目标位姿并进行规划
  move_group_->setPoseTarget(target_pose);
  bool success = (move_group_->plan(current_plan_) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(this->get_logger(), "运动规划成功");
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = current_plan_.start_state;
    display_trajectory.trajectory.push_back(current_plan_.trajectory);
    if (debug_) {
      RCLCPP_DEBUG(this->get_logger(), "生成的轨迹路径点数量: %zu",
                   display_trajectory.trajectory.size());
    }
    pub_display_trajectory_->publish(display_trajectory);
  } else {
    RCLCPP_WARN(this->get_logger(), "运动规划失败");
  }
  return success;
}

bool ArmPlanNode::PlanToPose(const std::vector<double> &target_joint_pose) {

  if (target_joint_pose.size() != this->joints_num_) {
    RCLCPP_WARN(this->get_logger(), "目标关节姿态数量与预期不符，规划失败");
    return false;
  }

  // 获取当前关节状态
  moveit::core::RobotStatePtr current_robot_state =
      move_group_->getCurrentState();
  if (current_robot_state) {
    std::stringstream ss;
    current_robot_state->printStatePositions(ss);
    if (debug_) {
      RCLCPP_DEBUG(this->get_logger(), "当前机械臂状态 (关节位置): %s",
                   ss.str().c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "无法获取当前机械臂状态以记录初始状态");
  }
  if (debug_) {
    RCLCPP_DEBUG(this->get_logger(), "目标关节姿态: [%f, %f, %f, %f, %f, %f]",
                 target_joint_pose[0], target_joint_pose[1],
                 target_joint_pose[2], target_joint_pose[3],
                 target_joint_pose[4], target_joint_pose[5]);
  }
  RCLCPP_INFO(this->get_logger(), "规划到目标关节角度");

  // 设置目标关节姿态并进行规划
  move_group_->setJointValueTarget(target_joint_pose);
  bool success = (move_group_->plan(current_plan_) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(this->get_logger(), "运动规划成功");
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = current_plan_.start_state;
    display_trajectory.trajectory.push_back(current_plan_.trajectory);
    if (debug_) {
      RCLCPP_DEBUG(this->get_logger(), "生成的轨迹路径点数量: %zu",
                   display_trajectory.trajectory.size());
    }
    pub_display_trajectory_->publish(display_trajectory);
  } else {
    RCLCPP_WARN(this->get_logger(), "运动规划失败");
  }
  return success;
}

bool ArmPlanNode::MoveServo(moveit_servo::TwistCommand &command) {
  if (!servo_) {
    RCLCPP_WARN(this->get_logger(),
                "Moveit Servo 尚未初始化完成，无法进行伺服控制");
    return false;
  }

  // 发送速度指令
  auto robot_state =
      this->planning_scene_monitor_->getStateMonitor()->getCurrentState();
  servo_->setCommandType(moveit_servo::CommandType::TWIST);
  moveit_servo::KinematicState next_state =
      servo_->getNextJointState(robot_state, command);
  ExecuteServo(next_state);
  return true;
}

bool ArmPlanNode::SolveIK(const geometry_msgs::msg::Pose &target_pose,
                          std::vector<double> &joint_pose) {
  if (!move_group_) {
    RCLCPP_WARN(this->get_logger(),
                "MoveGroup 尚未初始化完成，无法进行 IK 求解");
    return false;
  }

  auto robot_model = move_group_->getRobotModel();
  const auto *jmg = robot_model->getJointModelGroup(planning_group_name_);
  if (!jmg) {
    RCLCPP_ERROR(this->get_logger(), "关节组 %s 不存在",
                 planning_group_name_.c_str());
    return false;
  }

  moveit::core::RobotState state(robot_model);
  state = *move_group_->getCurrentState();

  // 当前版本的正确调用方式：group + pose + timeout
  bool ok = state.setFromIK(jmg, target_pose,
                            0.1 // timeout 秒
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

bool ArmPlanNode::ExecutePlan() {
  RCLCPP_INFO(this->get_logger(), "执行运动计划");

  if (debug_) {
    RCLCPP_INFO(this->get_logger(), "在虚拟环境中运行");
    bool success = (move_group_->execute(current_plan_) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "运动执行成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "运动执行失败");
    }

    return success;
  } else {
    RCLCPP_INFO(this->get_logger(), "在真实环境中运行");
    const auto &trajectory = current_plan_.trajectory.joint_trajectory;
    const auto &points = trajectory.points;

    if (points.empty()) {
      RCLCPP_WARN(this->get_logger(), "规划出的轨迹没有路径点，无法执行。");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "开始发送轨迹点，共 %zu 个", points.size());
    rclcpp::Rate rate(100); // 以 100Hz 的频率检查是否到达下一个时间点
    auto start_time = this->now();

    for (const auto &point : points) {
      // 等待直到当前时间超过该路径点的时间戳
      while (rclcpp::ok() &&
             (this->now() - start_time) < point.time_from_start) {
        rate.sleep();
      }

      // 准备要发送给下位机的消息
      std_msgs::msg::Float32MultiArray pos_msg;
      std_msgs::msg::Float32MultiArray vel_msg;

      // 请根据你的下位机协议调整
      pos_msg.data.insert(pos_msg.data.end(), point.positions.begin(),
                          point.positions.end());
      if (!point.velocities.empty()) {
        vel_msg.data.insert(vel_msg.data.end(), point.velocities.begin(),
                            point.velocities.end());
      } else {
        // 如果轨迹中没有速度信息，可以填充0
        std::vector<float> zero_velocities(this->joints_num_, 0.0f);
        vel_msg.data.insert(vel_msg.data.end(), zero_velocities.begin(),
                            zero_velocities.end());
      }

      pub_target_joint_pose_->publish(pos_msg);
      pub_target_joint_vel_->publish(vel_msg);

      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "执行被中断");
        return false;
      }
    }
    RCLCPP_INFO(this->get_logger(), "轨迹执行完毕");
    return true;
  }
}

bool ArmPlanNode::ExecuteServo(moveit_servo::KinematicState &state) {
  // RCLCPP_INFO(this->get_logger(), "执行伺服控制");

  if (!servo_) {
    RCLCPP_WARN(this->get_logger(),
                "Moveit Servo 尚未初始化完成，无法进行伺服控制");
    return false;
  }

  auto status = this->servo_->getStatus();
  if (status != moveit_servo::StatusCode::NO_WARNING && 
      status != moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY &&
      status != moveit_servo::StatusCode:: DECELERATE_FOR_LEAVING_SINGULARITY
    ) {
    RCLCPP_WARN(this->get_logger(), "Moveit Servo 状态异常，当前状态码: %d",
                static_cast<int>(status));
    RCLCPP_INFO(this->get_logger(), "pos: [%f, %f, %f, %f, %f, %f]",
                state.positions[0], state.positions[1], state.positions[2],
                state.positions[3], state.positions[4], state.positions[5]);
    return false;
  }
  if (this->debug_) {
    RCLCPP_INFO(this->get_logger(), "在虚拟环境中运行伺服控制");
    // 更新规划场景中的机械臂状态

    // auto current_state =
    //     planning_scene_monitor_->getStateMonitor()->getCurrentState();
    // if (!current_state) {
    //   RCLCPP_WARN(get_logger(), "获取当前状态失败");
    //   return false;
    // }
    // current_state->setJointGroupPositions(planning_group_name_,
    //                                       state.positions);
    RCLCPP_INFO(this->get_logger(), "pos: [%f, %f, %f, %f, %f, %f]",
                state.positions[0], state.positions[1], state.positions[2],
                state.positions[3], state.positions[4], state.positions[5]);
    planning_scene_monitor_->updateFrameTransforms();
    planning_scene_monitor_->updateSceneWithCurrentState();

    // 发布到假控制器
    if (!pub_fake_controller_) {
      RCLCPP_ERROR(get_logger(), "fake controller publisher 未初始化");
      return false;
    }
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    traj.joint_names = move_group_->getActiveJoints();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(state.positions.begin(), state.positions.end());
    point.velocities.assign(state.velocities.begin(), state.velocities.end());
    point.time_from_start = rclcpp::Duration::from_seconds(
        servo_params_.publish_period);
    traj.points.push_back(point);

    pub_fake_controller_->publish(traj);
  } else {
    // RCLCPP_INFO(this->get_logger(), "在真实环境中运行伺服控制");
    // 准备要发送给下位机的消息
    std_msgs::msg::Float32MultiArray pos_msg;
    std_msgs::msg::Float32MultiArray vel_msg;

    // 请根据你的下位机协议调整
    pos_msg.data.insert(pos_msg.data.end(), state.positions.begin(),
                        state.positions.end());
    vel_msg.data.insert(vel_msg.data.end(), state.velocities.begin(),
                        state.velocities.end());

    pub_target_joint_pose_->publish(pos_msg);
    pub_target_joint_vel_->publish(vel_msg);

    return true;
  }

  return true;
}
void ArmPlanNode::TargetEndPoseCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() != 7) {
    RCLCPP_WARN(this->get_logger(),
                "接收到的目标末端位姿数量与预期不符，忽略该消息");
    return;
  }
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id =
      "base_link"; // 根据具体机械臂的基座链接名称进行修改
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

void ArmPlanNode::TargetJointPoseCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() != this->joints_num_) {
    RCLCPP_WARN(this->get_logger(),
                "接收到的目标关节姿态数量与预期不符，忽略该消息");
    RCLCPP_WARN(this->get_logger(), "Expected %d, but got %zu",
                this->joints_num_, msg->data.size());
    return;
  }
  std::vector<double> target_joint_pose(msg->data.begin(), msg->data.end());
  bool plan_success = this->PlanToPose(target_joint_pose);
  if (!plan_success) {
    RCLCPP_WARN(this->get_logger(), "根据接收到的目标关节姿态规划失败");
  }
  ExecutePlan();
}

void ArmPlanNode::JointStateCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (this->debug_) {
    // 调试模式下不转发关节状态
    return;
  }
  if (msg->data.size() != this->joints_num_) {
    RCLCPP_WARN(this->get_logger(),
                "接收到的关节状态数量与预期不符，忽略该消息");
    RCLCPP_WARN(this->get_logger(), "Expected %d, but got %zu",
                this->joints_num_, msg->data.size());
    return;
  }

  // 检查 MoveGroup 是否已初始化
  if (!move_group_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "MoveGroup 尚未初始化，无法发布关节状态");
    return;
  }

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.name.resize(this->joints_num_);
  joint_state_msg.position.resize(this->joints_num_);
  // TODO: 根据具体机械臂的关节命名规则进行修改
  const std::vector<std::string> &joint_names = move_group_->getActiveJoints();
  if (joint_names.size() != this->joints_num_) {
    RCLCPP_ERROR_ONCE(
        this->get_logger(),
        "MoveGroup中的关节数量 (%zu) 与参数 joints_num (%d) 不匹配",
        joint_names.size(), this->joints_num_);
    return;
  }
  joint_state_msg.name = joint_names;
  joint_state_msg.position.assign(msg->data.begin(), msg->data.end());
  // 发布关节状态消息
  // RCLCPP_INFO(this->get_logger(), "Publishing joint states:");
  pub_joint_state_->publish(joint_state_msg);
}

void ArmPlanNode::JoyDataCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != 4) {
    RCLCPP_WARN(this->get_logger(),
                "接收到的手柄数据数量与预期不符，忽略该消息");
    return;
  }
  moveit_servo::TwistCommand twist_command;
  twist_command.frame_id = "base_link"; // 根据具体机械臂的基座链接名称进行修改
  twist_command.velocities.setZero();
  twist_command.velocities[0] = msg->data[1]; // x方向线速度
  twist_command.velocities[1] = msg->data[3]; // y方向线速度
  this->MoveServo(twist_command);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<ArmPlanNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
