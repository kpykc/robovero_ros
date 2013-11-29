# Robovero ROS nodes

First set environment.
Ensure that you have an access to port `/dev/ttyACM0` in Linux. For Debian/Ubuntu your user
should belong to dialout group.

To run code use next example (choose _use_mag:=true or false):

~~~{.bash}
roscore
rosrun robovero_ros ros_imu.py
rosrun nodelet nodelet standalone imu_filter_madgwick/ImuFilterNodelet /imu/data_raw:=/robovero/imu/data _use_mag:=false
rosrun nodelet nodelet standalone imu_filter_madgwick/ImuFilterNodelet /imu/data_raw:=/robovero/imu/data
rosrun rviz rviz #set config
~~~

## modemmanager issue

To disable ModemManager tries to establish connection using Robovero, add Robovero to blacklist
in `/etc/udev/rules.d/77-mm-robovero-blacklist.rules`.


~~~
ACTION!="add|change", GOTO="mm_usb_device_blacklist_end"
SUBSYSTEM!="usb", GOTO="mm_usb_device_blacklist_end"
ENV{DEVTYPE}!="usb_device",  GOTO="mm_usb_device_blacklist_end"

# Gumstix Overo/Robovero
# FT2232C Dual USB-UART/FIFO IC
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ENV{ID_MM_DEVICE_IGNORE}="1"
# NXP Semiconductors LPC1769
ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="2002", ENV{ID_MM_DEVICE_IGNORE}="1"

LABEL="mm_usb_device_blacklist_end"
~~~

~~~{.bash}
sudo nano /etc/udev/rules.d/77-mm-robovero-blacklist.rules
sudo service udev stop
sudo service udev start
sudo service modemmanager restart
~~~

