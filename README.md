# gazebo_continuous_track_example

## Tested environments
* Gazebo7 + ROS kinetic + Ubuntu 16.04
* Gazebo9 + ROS melodic + Ubuntu 18.04

## Set up
1. Update Gazebo from the OSRF apt repository ([ref](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install))
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrage
```
1. Clone relative packages
```
cd <your-ros-ws>/src
git clone git@github.com:yoshito-n-students/gazebo_continuous_track.git
git clone git@github.com:yoshito-n-students/gazebo_continuous_track_example.git
```
1. Build
```
catkin_make
```

## Run examples
1. Spawn tracked vehicles in Gazebo
```
cd <your-ros-ws>
source devel/setup.bash
roslaunch gazebo_continuous_example example_track_all_world.launch
```
1. (On another terminal) Send a velocity command to the vehicles
```
rostopic pub /cmd_vel std_msgs/Float64 "data: 0.5"
```