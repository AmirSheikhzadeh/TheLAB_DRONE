# Drone Tutorial

## First install 
- [Gazebo Old](https://wiki.ros.org/simulator_gazebo?distro=fuerte/)/ [Gazebo new](https://wiki.ros.org/gazebo_ros_pkgs) 
en [Rviz](http://wiki.ros.org/rviz)/ [User guide Rviz](http://wiki.ros.org/rviz/UserGuide) Ignore this step -> For documentation
- Install Gazebo5 [here](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=5.0) use the step by step installation
- First run 

```bash
sudo aptitude install libdrm-dev
```
- Then install Rviz [here](http://wiki.ros.org/rviz/UserGuide) in user guide with Indigo
## Install ardrone autonomy
```bash
sudo apt-get install ros-indigo-ardrone-autonomy
```
## Install joy_node and ardrone_joystick packages
```bash
# cd into ros root dir
roscd

# clone repository
svn checkout https://svncvpr.informatik.tu-muenchen.de/cvpr-ros-pkg/trunk/ardrone_helpers

# add to ros path (if required)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ardrone_helpers

# build package
rosmake ardrone_joystick
rosmake joy
```
## Install tum_simulator package:

```bash
# cd into ros root dir
roscd

# clone repository
git clone https://github.com/tum-vision/tum_simulator.git

# add to ros path (if required)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/tum_simulator

# build package
rosmake cvg_sim_gazebo_plugins
rosmake message_to_tf
```

## Use eender van de enivronnemets

```bash
roslaunch cvg_sim_test 3boxes_room.launch

roslaunch cvg_sim_test outdoor_flight.launch

roslaunch cvg_sim_test tum_kitchen.launch

roslaunch cvg_sim_test tum_kitchen_with_marker.launch

roslaunch cvg_sim_test garching_kitchen.launch

roslaunch cvg_sim_test garching_kitchen_with_marker.launch

roslaunch cvg_sim_test wg_collada.launch

roslaunch cvg_sim_test competition.launch
```
### Vindt [hier](http://wiki.ros.org/action/fullsearch/tum_simulator?action=fullsearch&context=180&value=linkto%3A%22tum_simulator%22) hoe ardrone gebruiken met simulator