# gazebo_continuous_track_example

## What's this?
Example of a realtime continuous track simulation for the Gazebo simulator & ROS. The blue and green ones are based on the proposed method with or without grousers. The red one is based on a conventional method by multiple wheels.

[![YouTube thumbnail](https://img.youtube.com/vi/0bPqNbOKPuQ/0.jpg)](https://www.youtube.com/watch?v=0bPqNbOKPuQ)

## Citation
> Y. Okada, S. Kojima, K. Ohno and S. Tadokoro, "Real-time Simulation of Non-Deformable Continuous Tracks with Explicit Consideration of Friction and Grouser Geometry," 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020

## Tested environments
* Gazebo7 + ROS Kinetic + Ubuntu 16.04
* Gazebo9 + ROS Melodic + Ubuntu 18.04

## Set up
1. Update Gazebo from the OSRF apt repository ([ref](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install))
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrage
```
2. Clone relative packages
```
cd <your-ros-ws>/src
git clone git@github.com:yoshito-n-students/gazebo_continuous_track.git
git clone git@github.com:yoshito-n-students/gazebo_continuous_track_example.git
```
3. Build
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
2. (On another terminal) Send a velocity command to the vehicles
```
rostopic pub /cmd_vel std_msgs/Float64 "data: 0.5"
```