#ifndef ARM_PLAN_NODE_HPP
#define ARM_PLAN_NODE_HPP

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


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
  bool ExecutePlan();


private:


  //回调函数
  void TargetEndPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void TargetJointPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  void JointStatusCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  //发布订阅接口
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_target_end_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_target_joint_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_joint_status_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr pub_display_trajectory_;

  //Moveit规划相关
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  //外部需传入参数
  bool debug_;
  std::string planning_group_name_;  //进行规划的目标robot_description
  std::string end_effector_link_name_; //进行规划的末端执行器link名称

  
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_; // 当前运动计划
};



#endif // ARM_PLAN_NODE_HPP