#ifndef ARM_PLAN_NODE_HPP
#define ARM_PLAN_NODE_HPP

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"



class ArmPlanNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   * 
   * @param options 节点配置参数
   */
  ArmPlanNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  //运动规划函数(支持外部调用)
  bool PlanToPose(const geometry_msgs::msg::PoseStamped& target_pose);
  bool PlanToPose(const std::vector<double>& target_joint_pose);
  bool SolveIK(const geometry_msgs::msg::Pose& target_pose,std::vector<double>& joint_pose);
  bool ExecutePlan();


private:


  //回调函数
  void TargetEndPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void TargetJointPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void JointStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  //发布订阅接口
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_target_end_pose_;  //接受目标末端位姿指令
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_target_joint_pose_; //接受目标关节姿态指令
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_state_;   //接受来自下位机的关节状态
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr pub_display_trajectory_; //发布规划轨迹
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_target_joint_pose_; //发布给下位机的目标关节姿态
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_target_joint_vel_;  //发布给下位机的目标关节速度
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;  //转发给robot_description的关节状态用于实时更新rviz与机器人模型

  //Moveit规划相关
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  //外部需传入参数
  bool debug_;
  uint8_t joints_num_;
  std::string planning_group_name_;  //进行规划的目标robot_description
  std::string end_effector_link_name_; //进行规划的末端执行器link名称

  
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_; // 当前运动计划
};



#endif // ARM_PLAN_NODE_HPP