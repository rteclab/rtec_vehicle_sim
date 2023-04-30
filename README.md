<h2 align="center">Vehicle simulation with Gazebo & ROS 2</h2>

### 1. Setup 
- This repository includes a virtual simulation of an autonomous vehicle that works in [Gazebo](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) simulator and ROS 2.
- Before building this repository, make sure that your system has ROS 2 packages installed on it (see this [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#) for ROS 2 installation). It is tested in ROS 2 foxy, but it will work on other distributes of ROS 2.
- In addition to ROS 2 packages, [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and [nav2](https://github.com/ros-planning/navigation2) packages installation are required.

### 2. Building a map
1) Launch gazebo simulator with a vehicle model and rviz2.
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

3) Set a goal on rviz.
    - Now, it's ready to navigate your vehicle autonomously. Set a goal by using `2D Goal Pose` button on rviz. Then, the vehicle will move there.
    




