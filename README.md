<h2 align="center">Vehicle simulation with Gazebo & ROS 2</h2>

### 1. Setup 
- This repository includes a virtual simulation of an autonomous vehicle that works in [Gazebo](http://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install) simulator and ROS 2.
- Before building this repository, make sure that your system has ROS 2 packages installed on it (see this [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#) for ROS 2 installation). It is tested in ROS 2 foxy & galactic versions, but it will work on other ROS 2 distributes.
- In addition to ROS 2 packages, [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), [nav2](https://github.com/ros-planning/navigation2), and [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) installations are required.
    - For source installations, please visit the above links.
    - For binary installations of `nav2` & `slam_toolbox` using OS package manager:
        ```
        # nav2 package
        sudo apt install ros-<ros2-distro>-navigation2
        sudo apt install ros-<ros2-distro>-nav2-bringup

        # slam_toolbox
        sudo apt install ros-<ros2-distro>-slam-toolbox
        ```
- Make workspace and clone the repository:
    ```
    mkdir -p ~/nav2_ws/src/
    cd ~/nav2_ws/src/
    git clone https://github.com/rteclab/rtec_vehicle_sim.git
    ```
- Before building the sources, some packages are needed to be installed to meet dependencies such as [camera_info_manager](https://github.com/ros-perception/image_common) and [xacro](https://github.com/ros/xacro). Your system may require other packages depending on the current status. 

- Then, build the workspace.
    ```
    cd ~/nav2_ws
    colcon build --symlink-install
    ```

### 2. Building a map
1) Launch gazebo simulator with a vehicle model and rviz2.
    - Before running the simulator and rviz2, change the model path to the absolute location of the the robot model by modifying
        * Description File: /home/**${USER}**/nav2_ws/src/rtec_vehicle_sim/models/vehicle_rviz.xacro in `/config/rtec_sim.rviz` file.
    - Then, launch simulation as follow:
        ```
        ros2 launch rtec_vehicle_sim all.launch.py
        ```
    Notes: 
    - vehicle model for gazebo : `/models/vehicle.xacro`
    - vehicle model for rviz2 : `/models/vehicle_rviz.xacro`
    
2) Launch `slam_toolbox` to build a map of the world.
    - First, change the directory where you want to save a map.
        ```
        cd ~/nav2_ws/src/rtec_vehicle_sim/maps/
        ```
    - Then, launch `online_async` SLAM of `slam_toolbox`.
        ```
        ros2 launch slam_toolbox online_async_launch.py params_file:=../config/mapper_params_online_async.yaml use_sim_time:=true
        ```
        Or, you can run the following script.
        ```
        ./mapping.sh
        ```
        This package launches `map` frame and publishes a transform from `map` to `odom`.
        * Notes: you can check the entire tf trees by running a script (`./tf_view.sh`) on `/tf_views` directory.
3) Launch keyboard control to move the vehicle.
    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    Then, move the vehicle around the world to generate a map.
4) Save a map file.
    - Use a `slamToolboxPlugin` on rviz to save a map file.
    - On rviz, click `Panels -> Add New Panel -> SlamToolboxPlugin`.
    - Write a your map name on the blank of `Save Map` and `Serialize Map`, then click the button to save it.

### 3. Navigation
- First, change the directory where you want to save a map.
    ```
    cd ~/nav2_ws/src/rtec_vehicle_sim/maps/
    ```
    
1) Launch a `map_server`.
    - `nav2_map_server` package, which is a part of `nav2` stack, will publish a `map` topic by using a map we built on the above.
        ```
        ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map_save.yaml -p use_sim_time:=true
        ```
        - Note: alternatively, you can run a script (`run_map_server.sh`) on `/maps` directory.
    - This map server needs an activation to operate it. So, run the following command in a different terminal.
        ```
        ros2 run nav2_util lifecycle_bringup map_server
        ```
        - Note: alternatively, you can run a script (`run_lifecycle_map_server.sh`) on `/maps` directory.

2) Launch localization package.
    - We need run a localization package such as `slam_toolbox` localization, which we will use here. Or, you may use a different localization algorithm such as `amcl` localization, a part of `nav2` stack.
    - The following command uses `slam_toolbox` localization. ***Make sure that you need put a right map path `map_file_name` on `mapper_params_online_async_localisation.yaml` file.***
        ```
        ros2 launch slam_toolbox online_async_launch.py params_file:=../config/mapper_params_online_async_localisation.yaml use_sim_time:=true
        ```
3) Launch navigation package.
    - We run navigation package from `nav2` stack.
        ```
        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
        ```

4) Set a goal on rviz.
    - Now, it's ready to navigate your vehicle autonomously. Set a goal by using `2D Goal Pose` button on rviz. Then, the vehicle will move there.
    




