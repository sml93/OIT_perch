** TO RUN SIMULATION ONLY **

Prerequisites:
Make sure you have installed Ubuntu 18.04 and above.


INSTALLATION:
Install ROS-desktop-full-> Visit ROSwiki for documentation:
- ensure that you install ROS 1 (not ROS 2)
- ensure that Gazebo is installed along the full desktop installation.
- ensure to setup catkin_ws too


Install matlab for Linux


Installing mavros:
follow the documentation in https://docs.px4.io/main/en/ros/mavros_installation.html


Installing PX4:
git clone https://github.com/PX4/PX4-Autopilot --recursive
git checkout tags/v1.13.1
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
git submodule update --init --recursive
git config --global --unset https.proxy (if cannot install some packages from github)


Installing gz_ros_plugin:
fork and git clone https://github.com/sml93/gz_ros_plugin.git into ~/catkin_ws/src
git checkout -branch fourth  ## this contains the plugin with ceiling effect.
catkin build
source catkin_ws/devel/setup.bash


Setting up environment:
fork and git clone https://github.com/sml93/OIT_perch.git 

cd OIT_perch/perchControl/PX4-Autopilot_utils/
copy posix_sitl_OIT.launch into PX4-Autopilot/launch
copy iris_OIT2.sdf into PX4-Autopilot/Tools/sitl_gazebo/models/iris/
copy empty_OIT.world into X4-Autopilot/Tools/sitl_gazebo/worlds


Optional (helper tools):
fork and git clone https://github.com/sml93/oit_dyn_rec.git into ~/catkin_ws/src
catkin build




EXECUTE SIMULATION:
1) launch matlab
2) roslaunch px4 posix_sitl_OIT.launch
3) roslaunch mavros px4.launch fcu_url:="udp://:14540@172.22.41.222"
4) cd ~/OIT_perch/perchControl/ and run python mavrosAttCtrl_OIT.py
5) python3 flcTC.py
6) rosrun oit_dyn_rec server.py
7) rosrun rqt_reconfigure rqt_reconfigure, select the oit_topic to change the orientation and jetforce at will. Safety switch via mission_status: boolean type. Make sure this is False before next flight. 



