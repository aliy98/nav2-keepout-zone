# nav2-keepout-zone
Navigation in ROS2 with [Nav2](https://navigation.ros.org/) introduced a lot of new features and possibilities. The goal of this assignment is to explore and test the new feature 
of navigating in a known environment while avoiding user-defined [keep out areas](https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html).

## Installation
This package requires the following dependencies:
```
sudo apt install ros-<ros2-distro>-slam-toolbox
```
```
sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup '~ros-<distro>-turtlebot3-.*'
```
Once all the dependencies are met, the packge could be cloned and compiled as it is represented:
```
mkdir -p colcon_ws/src
```
```
cd colcon_ws/src
```
```
git clone https://github.com/aliy98/nav2-keepout-zone
```
```
cd colcon_ws
```
```
colcon build --symlink-install
```
```
source install/setup.bash
```

## Simulation Environment
[Webots2023a](https://cyberbotics.com/doc/blog/Webots-2023-a-release) is a simulation software, in which this project was done. It is compliant with ROS2, 
and provides the [TIAGo](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_tiago) robot urdf model, 
which is chosen to be used in this project.

The following figure, provides an overview from the simulation environment of this project:
<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224727414-f7ecc39a-b3ee-4137-8293-a9902fe2817c.png" width="500" title="room">
</p>

## Mapping
The first step of this experiment, consists of creating a map of the indoor environemnt. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) provides an online
synchronous mapping ROS node, using the following command:
```
ros2 launch slam_toolbox online_async_launch.py
```
which was used in this project along with [Nav2](https://github.com/ros-planning/navigation2) applications. The following command would bringup navigation:
```
ros2 launch nav2_bringup navigation_launch.py
```

This would help us to define goal points during slam. In order to launch these two nodes along with the simulation environment, the `slam_launch.py` file
is provided in this package, which takes an `world` file as an argument:
```
ros2 launch sofar_assignment slam_launch.py world:=default.wbt
```
Here is an overview of the mapping process:
<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224721685-47e7dda6-6dbe-4502-abe1-cb4a5b12ca88.gif" width="800" title="slam">
</p>

The UML component diagram of this part is shown in following figure:
<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224717872-95fbb499-cbaf-447b-9cfe-4c1b159d912e.png" width="500" title="sofar_slam">
</p>

## Creating keepout filter masks:
Once the map is built, the keepout areas could be drwan on the `map.pgm` file and saved as the mask filter. The corresponding mask yaml file mode has to be chosen as scale,
so that the occupancy grids of each area, would be defined with a certain level of occupancy.
The follwoing figures show a sample of keepout filter mask along with the original map file:

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224722273-0187223b-b5bf-4937-84eb-e0e4037564f2.jpg" width="300" title="map">
</p>

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224722411-aba56d08-a549-4d9e-8236-fda1e1a588e3.jpg" width="300" title="mask">
</p>



## Navigation and path planning 
In the last step of this project, the navigation and path planning part is done, considering the keepout zones. The provided `nav_launch.py` in this package
would bring the simulation environment along with navigation2 applications such as `map_server`, `costmap_filter_info_server` and `life_cycle_manager`. The following
command represents the arguments that this launch file requires:
```
ros2 launch sofar_assignment nav_launch.py world:=default.wbt map:=src/nav2-keepout-zone/resource/map.yaml params_file:=src/nav2-keepout-zone/resource/nav2_params.yaml keepout_params_file:=src/nav2-keepout-zone/resource/keepout_params.yaml mask:=src/nav2-keepout-zone/resource/keepout_mask.yaml
```

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224722217-fa4e5a80-33f5-47ea-abfc-539d894df944.gif" width="800" title="nav">
</p>

The UML component diagram of this part is shown in following figure:
<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/224723034-963d4be0-ebec-419a-af82-fe3d3647da7a.png" width="500" title="sofar_nav">
</p>

## Authors and Contacts
- Ali Yousefi
- email: aliyousefi98@outlook.com 


