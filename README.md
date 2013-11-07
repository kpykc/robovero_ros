# Robovero ROS nodes

First set environment.

To run code use next example (choose _use_mag:=true or false):

~~~{.bash}
roscore
rosrun robovero_ros ros_imu.py
rosrun nodelet nodelet standalone imu_filter_madgwick/ImuFilterNodelet /imu/data_raw:=/robovero/imu/data _use_mag:=false
rosrun nodelet nodelet standalone imu_filter_madgwick/ImuFilterNodelet /imu/data_raw:=/robovero/imu/data
rosrun rviz rviz #set config
~~~
