#bin bash
conda deactivate
source ~/Engineer_IKF/install/setup.bash
ros2 topic pub --once /target_joint_pose std_msgs/Float32MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [-1.497017, -0.447303, 1.079093, -0.165643, 1.058486, 1.613602]}"