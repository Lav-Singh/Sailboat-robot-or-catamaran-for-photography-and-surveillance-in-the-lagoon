<h1>Sailbot robot or catamaran for photography and surveillance in the lagoon</h1></br>

Implementation of a control system for a USV meant for lagoon surveillance in VRX Gazebo</br>

<h2>Contents</h2></br>
The package contains 3 nodes : Destination(s), Wayfinding, Thruster Control.</br>

Destination node : publishes the waypoint(s) that the USV needs to get to.</br>
Wayfinding node : subscribes to the Destination node and computes angular and linear velocity and publishes it to a CMD_VEL.</br>
Thruster Control node : subscribes to CMD_VEL and computes the differential thrust so that the USV reaches its current destination.</br>

Simulation environment : </br>
VRX (classic) : https://github.com/osrf/vrx </br>
Ros Noetic </br>
Ubuntu 20.4 LTS </br>
All was installed and developped on Windows Subsystem for Linux 2(WSL2) on a windows 10 machine [AMD ryzen 5 3600, 16GB RAM, 1TB SSD, RTX 3060] </br>

To use the packages:</br>
wamv_control => create a workspace using catkin </br>
                create a package with dependencies rospy,roscpp,std_msgs, sensor_msgs</br>
                copy and paste everything from wamv_control/src to the package</br>
my_wamv => contains URDF of a double-thruster WAMV</br>
            to use it : roslaunch vrx_gazebo sydneyregatta.launch urdf:=<Path_to_folder>/my_wamv/my_wamv.urdf</br>
