#bin bash
conda deactivate
source ~/Engineer_IKF/install/setup.bash
ros2 topic pub --once /target_joint_pose std_msgs/Float32MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"