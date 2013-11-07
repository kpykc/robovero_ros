# Robovero ROS nodes

roscore
rosrun nodelet nodelet standalone imu_filter_madgwick/ImuFilterNodelet /imu/data_raw:=/robovero/imu/data _use_mag:=false
rosrun nodelet nodelet standalone imu_filter_madgwick/ImuFilterNodelet /imu/data_raw:=/robovero/imu/data
rosrun rviz rviz #set config
