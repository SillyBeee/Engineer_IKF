#bin bash
conda deactivate
source ~/Engineer_IKF/install/setup.bash
ros2 topic pub --once /target_joint_pose std_msgs/Float32MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}"