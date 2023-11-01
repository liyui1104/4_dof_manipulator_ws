# 4_dof_manipulator


> Preferences: 
1 ros\_control: [ROS Control | ROS Robotics (rosroboticslearning.com)](https://www.rosroboticslearning.com/ros-control)

> 2 moveit: [https://blog.csdn.net/qq\_38288618/article/details/78078514?utm\_medium=distribute.pc\_relevant.none-task-blog-2\~default\~baidujs\_baidulandingword\~default-0-78078514-blog-80300069.pc\_relevant\_3mothn\_strategy\_and\_data\_recovery&spm=1001.2101.3001.4242.1&utm\_relevant\_index=3](https://blog.csdn.net/qq_38288618/article/details/78078514?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-78078514-blog-80300069.pc_relevant_3mothn_strategy_and_data_recovery&spm=1001.2101.3001.4242.1&utm_relevant_index=3)

> 3 manipulator: [bandasaikrishna (github.com)](https://github.com/bandasaikrishna)



# **1 Architecture**
* **The expected functionality**

**1) Given a three-dimensional coordinate point relative to the base\_link frame of reference, the end-effector of the manipulator can move to it.**

---
* **The workflow of controlling my manipulator arm using moveit and ros\_control**

![image](https://gitee.com/lingyu-i/4_dof_manipulator_ws/raw/master/markdown_image/drawio_workflow.png)


1. C++ API, Rviz

Users will send the target pose(a 3D coordinate point based on the manipulator end-effector coordinate system) to MoveIt through C++ API or Rviz GUI. It's the input of the entire workflow.

2. MoveIt, Action Client(move\_group)

MoveIt will calculate the trajectory depending on the configured kinematics plug-in(KDL, IKfast and etc). And then, MoveIt will give the planned trajectory to the controller of ros\_control.

3. Action Server(ros\_control)

The controller of ros\_control will publish interpolated trajectory via a topic named `/lower_controller/command`.

All work will be done by a class named arm\_controller\_hardware\_interface(it inherits from hardware\_interface::RobotHW).

4. rosserial\_arduino

The rosserial\_arduino will transmit the data of the topic(`/lower_controller/command`) to Arduino via serial port(`/tty/USB0`).

5. Arduino(subscriber), Stepper Motor

The Arduino will subscribe the topic(`/lower_controller/command`), and then it will translate the command(radians of joints) into number of pulses. Finally, stepper motors will work according to given number of pulses.

It should be noted that there're not sensors which can feedback current position here. **So we assume control commands are executed ideally.**

---
* **The workflow of controlling the robot using ros\_control**

![image](https://gitee.com/lingyu-i/4_dof_manipulator_ws/raw/master/markdown_image/iZPgncoNh8YUNNNISRUBySwmlmyvbUHXCphnH4HIllM.png)

* **The boot sequence for myarm.launch**

![image](https://gitee.com/lingyu-i/4_dof_manipulator_ws/raw/master/markdown_image/GmoAiECkPao3zzfBOGguP8Gfcmob1ugkLiqqMJexuaU.png)

* **The rqt\_graph**

![image](https://gitee.com/lingyu-i/4_dof_manipulator_ws/raw/master/markdown_image/g-XU2uPsXHXpFWgxVfnapsNbn5DexiZOp9D7XKFHxEI.png)

---
* **Note:  JointTrajectoryController is essentially actionserver, which drives the mechanical arm, it will interact with the client of Moveit.**

// The hardware\_interface::RobotHW class is derived from InterfaceManger. The InterfaceManger class is an Interface management class. It provides the registerInterface function to register the interface class of the robot hardware. 

# **2 The inplementation process**
## **2.1 ros\_control**
### **2.1.1 arm\_controller\_hardware\_interface class**
Write a class named arm\_controller\_hardware\_interface that inherits from hardware\_interface::RobotHW. This class will include all methods to control my robot arm, and I will achieve all functions in it.

**I will only place the complete code here, the code interpretation of ros\_control is in this note.** [How to use ros_control on your robot-ros_control_usage-ros_control-ros_灵雨i的博客-CSDN博客](https://blog.csdn.net/weixin_66362646/article/details/128561711)



## **2.2 Arduino code**
* NOTE

The radian to the Arduino should **not exceed 31.415926**.

The value given to the command array ranges **from -2147483648 to 2147483647**.

The radian given to Arduino is multiplied by 106 first, then sent to Arduino through rosserial\_arduino. And then it divided by 106  in the Arduino code to ensure the precision of the control command.



**The data of `/lower_controller/command` is the radian to be reached by the joint (accuracy 0.000001).** 

**The time for updating `/lower_controller/command` is 0.1 seconds, and the time for executing command by the MCU is 0.09 seconds (to make up for the time difference). **



The type of the message for communication is std\_msgs/Int32MultiArray.

> How to publish and subscribe arrays in ROS: [https://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/](https://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/)



# **3 Debug information**
* The rqt\_graph

![image](https://gitee.com/lingyu-i/4_dof_manipulator_ws/raw/master/markdown_image/Smqgv_NmR7r6qWi0ruWEFsHBCfQkbQrjuk-O0wcHvJo.png)



* The output of myarm.launch

```bash
liyui@liyui:~/4_dof_manipulator_ws$ roslaunch myarm_moveit_config myarm.launch
... logging to /home/liyui/.ros/log/65225148-a464-11ed-8b30-a3267de56412/roslaunch-liyui-9250.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://liyui:45211/

SUMMARY
========

PARAMETERS
 * /arm_position_controller/gains/joint1/d: 0.1
 * /arm_position_controller/gains/joint1/i: 0.0
 * /arm_position_controller/gains/joint1/p: 100.0
 * /arm_position_controller/gains/joint2/d: 0.1
 * /arm_position_controller/gains/joint2/i: 0.0
 * /arm_position_controller/gains/joint2/p: 100.0
 * /arm_position_controller/gains/joint3/d: 0.1
 * /arm_position_controller/gains/joint3/i: 0.0
 * /arm_position_controller/gains/joint3/p: 100.0
 * /arm_position_controller/gains/joint4/d: 0.1
 * /arm_position_controller/gains/joint4/i: 0.0
 * /arm_position_controller/gains/joint4/p: 100.0
 * /arm_position_controller/joints: ['joint1', 'joint...
 * /arm_position_controller/type: position_controll...
 * /generic_hw_control_loop/cycle_time_error_threshold: 0.01
 * /generic_hw_control_loop/loop_hz: 300
 * /hardware_interface/joints: ['joint1', 'joint...
 * /joint_state_controller/publish_rate: 50
 * /joint_state_controller/type: joint_state_contr...
 * /move_group/allow_trajectory_execution: True
 * /move_group/capabilities: 
 * /move_group/controller_list: [{'name': 'arm_po...
 * /move_group/default_planning_pipeline: ompl
 * /move_group/disable_capabilities: 
 * /move_group/max_range: 5.0
 * /move_group/monitor_dynamics: False
 * /move_group/moveit_controller_manager: moveit_simple_con...
 * /move_group/moveit_manage_controllers: True
 * /move_group/octomap_resolution: 0.025
 * /move_group/planning_pipelines/chomp/collision_clearance: 0.2
 * /move_group/planning_pipelines/chomp/collision_threshold: 0.07
 * /move_group/planning_pipelines/chomp/enable_failure_recovery: False
 * /move_group/planning_pipelines/chomp/jiggle_fraction: 0.05
 * /move_group/planning_pipelines/chomp/joint_update_limit: 0.1
 * /move_group/planning_pipelines/chomp/learning_rate: 0.01
 * /move_group/planning_pipelines/chomp/max_iterations: 200
 * /move_group/planning_pipelines/chomp/max_iterations_after_collision_free: 5
 * /move_group/planning_pipelines/chomp/max_recovery_attempts: 5
 * /move_group/planning_pipelines/chomp/obstacle_cost_weight: 1.0
 * /move_group/planning_pipelines/chomp/planning_plugin: chomp_interface/C...
 * /move_group/planning_pipelines/chomp/planning_time_limit: 10.0
 * /move_group/planning_pipelines/chomp/pseudo_inverse_ridge_factor: 1e-4
 * /move_group/planning_pipelines/chomp/request_adapters: default_planner_r...
 * /move_group/planning_pipelines/chomp/ridge_factor: 0.0
 * /move_group/planning_pipelines/chomp/smoothness_cost_acceleration: 1.0
 * /move_group/planning_pipelines/chomp/smoothness_cost_jerk: 0.0
 * /move_group/planning_pipelines/chomp/smoothness_cost_velocity: 0.0
 * /move_group/planning_pipelines/chomp/smoothness_cost_weight: 0.1
 * /move_group/planning_pipelines/chomp/start_state_max_bounds_error: 0.1
 * /move_group/planning_pipelines/chomp/use_pseudo_inverse: False
 * /move_group/planning_pipelines/chomp/use_stochastic_descent: True
 * /move_group/planning_pipelines/ompl/arm/default_planner_config: RRT
 * /move_group/planning_pipelines/ompl/arm/longest_valid_segment_fraction: 0.005
 * /move_group/planning_pipelines/ompl/arm/planner_configs: ['AnytimePathShor...
 * /move_group/planning_pipelines/ompl/arm/projection_evaluator: joints(joint1,joi...
 * /move_group/planning_pipelines/ompl/jiggle_fraction: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/AnytimePathShortening/hybridize: True
 * /move_group/planning_pipelines/ompl/planner_configs/AnytimePathShortening/max_hybrid_paths: 24
 * /move_group/planning_pipelines/ompl/planner_configs/AnytimePathShortening/num_planners: 4
 * /move_group/planning_pipelines/ompl/planner_configs/AnytimePathShortening/planners: 
 * /move_group/planning_pipelines/ompl/planner_configs/AnytimePathShortening/shortcut: True
 * /move_group/planning_pipelines/ompl/planner_configs/AnytimePathShortening/type: geometric::Anytim...
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/balanced: 0
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/cache_cc: 1
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/extended_fmt: 1
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/heuristics: 1
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/nearest_k: 1
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/num_samples: 1000
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/optimality: 1
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/radius_multiplier: 1.0
 * /move_group/planning_pipelines/ompl/planner_configs/BFMT/type: geometric::BFMT
 * /move_group/planning_pipelines/ompl/planner_configs/BKPIECE/border_fraction: 0.9
 * /move_group/planning_pipelines/ompl/planner_configs/BKPIECE/failed_expansion_score_factor: 0.5
 * /move_group/planning_pipelines/ompl/planner_configs/BKPIECE/min_valid_path_fraction: 0.5
 * /move_group/planning_pipelines/ompl/planner_configs/BKPIECE/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/BKPIECE/type: geometric::BKPIECE
 * /move_group/planning_pipelines/ompl/planner_configs/BiEST/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/BiEST/type: geometric::BiEST
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/cost_threshold: 1e300
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/frountier_node_ratio: 0.1
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/frountier_threshold: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/init_temperature: 100
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/temp_change_factor: 0.1
 * /move_group/planning_pipelines/ompl/planner_configs/BiTRRT/type: geometric::BiTRRT
 * /move_group/planning_pipelines/ompl/planner_configs/EST/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/EST/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/EST/type: geometric::EST
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/cache_cc: 1
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/extended_fmt: 1
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/heuristics: 0
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/nearest_k: 1
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/num_samples: 1000
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/radius_multiplier: 1.1
 * /move_group/planning_pipelines/ompl/planner_configs/FMT/type: geometric::FMT
 * /move_group/planning_pipelines/ompl/planner_configs/KPIECE/border_fraction: 0.9
 * /move_group/planning_pipelines/ompl/planner_configs/KPIECE/failed_expansion_score_factor: 0.5
 * /move_group/planning_pipelines/ompl/planner_configs/KPIECE/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/KPIECE/min_valid_path_fraction: 0.5
 * /move_group/planning_pipelines/ompl/planner_configs/KPIECE/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/KPIECE/type: geometric::KPIECE
 * /move_group/planning_pipelines/ompl/planner_configs/LBKPIECE/border_fraction: 0.9
 * /move_group/planning_pipelines/ompl/planner_configs/LBKPIECE/min_valid_path_fraction: 0.5
 * /move_group/planning_pipelines/ompl/planner_configs/LBKPIECE/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/LBKPIECE/type: geometric::LBKPIECE
 * /move_group/planning_pipelines/ompl/planner_configs/LBTRRT/epsilon: 0.4
 * /move_group/planning_pipelines/ompl/planner_configs/LBTRRT/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/LBTRRT/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/LBTRRT/type: geometric::LBTRRT
 * /move_group/planning_pipelines/ompl/planner_configs/LazyPRM/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/LazyPRM/type: geometric::LazyPRM
 * /move_group/planning_pipelines/ompl/planner_configs/LazyPRMstar/type: geometric::LazyPR...
 * /move_group/planning_pipelines/ompl/planner_configs/PDST/type: geometric::PDST
 * /move_group/planning_pipelines/ompl/planner_configs/PRM/max_nearest_neighbors: 10
 * /move_group/planning_pipelines/ompl/planner_configs/PRM/type: geometric::PRM
 * /move_group/planning_pipelines/ompl/planner_configs/PRMstar/type: geometric::PRMstar
 * /move_group/planning_pipelines/ompl/planner_configs/ProjEST/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/ProjEST/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/ProjEST/type: geometric::ProjEST
 * /move_group/planning_pipelines/ompl/planner_configs/RRT/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/RRT/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/RRT/type: geometric::RRT
 * /move_group/planning_pipelines/ompl/planner_configs/RRTConnect/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/RRTConnect/type: geometric::RRTCon...
 * /move_group/planning_pipelines/ompl/planner_configs/RRTstar/delay_collision_checking: 1
 * /move_group/planning_pipelines/ompl/planner_configs/RRTstar/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/RRTstar/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/RRTstar/type: geometric::RRTstar
 * /move_group/planning_pipelines/ompl/planner_configs/SBL/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/SBL/type: geometric::SBL
 * /move_group/planning_pipelines/ompl/planner_configs/SPARS/dense_delta_fraction: 0.001
 * /move_group/planning_pipelines/ompl/planner_configs/SPARS/max_failures: 1000
 * /move_group/planning_pipelines/ompl/planner_configs/SPARS/sparse_delta_fraction: 0.25
 * /move_group/planning_pipelines/ompl/planner_configs/SPARS/stretch_factor: 3.0
 * /move_group/planning_pipelines/ompl/planner_configs/SPARS/type: geometric::SPARS
 * /move_group/planning_pipelines/ompl/planner_configs/SPARStwo/dense_delta_fraction: 0.001
 * /move_group/planning_pipelines/ompl/planner_configs/SPARStwo/max_failures: 5000
 * /move_group/planning_pipelines/ompl/planner_configs/SPARStwo/sparse_delta_fraction: 0.25
 * /move_group/planning_pipelines/ompl/planner_configs/SPARStwo/stretch_factor: 3.0
 * /move_group/planning_pipelines/ompl/planner_configs/SPARStwo/type: geometric::SPARStwo
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/degree: 16
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/estimated_dimension: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/max_degree: 18
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/max_pts_per_leaf: 6
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/min_degree: 12
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/min_valid_path_fraction: 0.2
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/type: geometric::STRIDE
 * /move_group/planning_pipelines/ompl/planner_configs/STRIDE/use_projected_distance: 0
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/frountierNodeRatio: 0.1
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/frountier_threshold: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/goal_bias: 0.05
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/init_temperature: 10e-6
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/k_constant: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/max_states_failed: 10
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/min_temperature: 10e-10
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/range: 0.0
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/temp_change_factor: 2.0
 * /move_group/planning_pipelines/ompl/planner_configs/TRRT/type: geometric::TRRT
 * /move_group/planning_pipelines/ompl/planning_plugin: ompl_interface/OM...
 * /move_group/planning_pipelines/ompl/request_adapters: default_planner_r...
 * /move_group/planning_pipelines/ompl/start_state_max_bounds_error: 0.1
 * /move_group/planning_pipelines/pilz_industrial_motion_planner/capabilities: pilz_industrial_m...
 * /move_group/planning_pipelines/pilz_industrial_motion_planner/default_planner_config: PTP
 * /move_group/planning_pipelines/pilz_industrial_motion_planner/planning_plugin: pilz_industrial_m...
 * /move_group/planning_pipelines/pilz_industrial_motion_planner/request_adapters: 
 * /move_group/planning_scene_monitor/publish_geometry_updates: True
 * /move_group/planning_scene_monitor/publish_planning_scene: True
 * /move_group/planning_scene_monitor/publish_state_updates: True
 * /move_group/planning_scene_monitor/publish_transforms_updates: True
 * /move_group/sense_for_plan/max_safe_path_cost: 1
 * /move_group/sensors: []
 * /move_group/trajectory_execution/allowed_execution_duration_scaling: 1.2
 * /move_group/trajectory_execution/allowed_goal_duration_margin: 0.5
 * /move_group/trajectory_execution/allowed_start_tolerance: 0.01
 * /robot_description: <?xml version="1....
 * /robot_description_kinematics/arm/kinematics_solver: kdl_kinematics_pl...
 * /robot_description_kinematics/arm/kinematics_solver_search_resolution: 0.005
 * /robot_description_kinematics/arm/kinematics_solver_timeout: 0.005
 * /robot_description_planning/cartesian_limits/max_rot_vel: 1.57
 * /robot_description_planning/cartesian_limits/max_trans_acc: 2.25
 * /robot_description_planning/cartesian_limits/max_trans_dec: -5
 * /robot_description_planning/cartesian_limits/max_trans_vel: 1
 * /robot_description_planning/default_acceleration_scaling_factor: 0.1
 * /robot_description_planning/default_velocity_scaling_factor: 0.1
 * /robot_description_planning/joint_limits/joint1/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/joint1/has_velocity_limits: True
 * /robot_description_planning/joint_limits/joint1/max_acceleration: 0
 * /robot_description_planning/joint_limits/joint1/max_velocity: 6.28
 * /robot_description_planning/joint_limits/joint2/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/joint2/has_velocity_limits: True
 * /robot_description_planning/joint_limits/joint2/max_acceleration: 0
 * /robot_description_planning/joint_limits/joint2/max_velocity: 6.28
 * /robot_description_planning/joint_limits/joint3/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/joint3/has_velocity_limits: True
 * /robot_description_planning/joint_limits/joint3/max_acceleration: 0
 * /robot_description_planning/joint_limits/joint3/max_velocity: 6.28
 * /robot_description_planning/joint_limits/joint4/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/joint4/has_velocity_limits: True
 * /robot_description_planning/joint_limits/joint4/max_acceleration: 0
 * /robot_description_planning/joint_limits/joint4/max_velocity: 6.28
 * /robot_description_semantic: <?xml version="1....
 * /rosdistro: noetic
 * /rosversion: 1.15.15

NODES
  /
    controller_spawner (controller_manager/spawner)
    move_group (moveit_ros_move_group/move_group)
    myarm_hardware_interface_node (myarm_hardware_interface/myarm_hardware_interface)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rosserial_arduino_node (rosserial_python/serial_node.py)
    rviz_liyui_9250_5513372120012256045 (rviz/rviz)

auto-starting new master
process[master]: started with pid [9258]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 65225148-a464-11ed-8b30-a3267de56412
process[rosout-1]: started with pid [9269]
started core service [/rosout]
process[myarm_hardware_interface_node-2]: started with pid [9276]
process[robot_state_publisher-3]: started with pid [9277]
process[controller_spawner-4]: started with pid [9278]
process[move_group-5]: started with pid [9279]
process[rviz_liyui_9250_5513372120012256045-6]: started with pid [9285]
process[rosserial_arduino_node-7]: started with pid [9295]
[ INFO] [1675498662.548474278]: Loading robot model 'myarm_description'...
[ INFO] [1675498662.549039820]: No root/virtual joint specified in SRDF. Assuming fixed joint
[ INFO] [1675498662.614468492]: rviz version 1.14.19
[ INFO] [1675498662.614511537]: compiled against Qt version 5.12.8
[ INFO] [1675498662.614530001]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1675498662.623295008]: Forcing OpenGl version 0.
[ INFO] [1675498662.738649530]: Publishing maintained planning scene on 'monitored_planning_scene'
[ INFO] [1675498662.744109301]: Listening to 'joint_states' for joint states
[ INFO] [1675498662.751706514]: Listening to '/attached_collision_object' for attached collision objects
[ INFO] [1675498662.751769874]: Starting planning scene monitor
[ INFO] [1675498662.756290413]: Listening to '/planning_scene'
[ INFO] [1675498662.756365994]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[ INFO] [1675498662.760454747]: Listening to '/collision_object'
[ INFO] [1675498662.765147214]: Listening to '/planning_scene_world' for planning scene world geometry
[INFO] [1675498662.787179]: Controller Spawner: Waiting for service controller_manager/load_controller
[INFO] [1675498662.790610]: Controller Spawner: Waiting for service controller_manager/switch_controller
[INFO] [1675498662.793128]: Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [1675498662.795377]: Loading controller: joint_state_controller
[ INFO] [1675498662.795710983]: Stereo is NOT SUPPORTED
[ INFO] [1675498662.795825595]: OpenGL device: llvmpipe (LLVM 9.0.1, 128 bits)
[ INFO] [1675498662.795864006]: OpenGl version: 3.1 (GLSL 1.4).
[ INFO] [1675498662.825138433]: Loading planning pipeline 'chomp'
[INFO] [1675498662.838993]: Loading controller: arm_position_controller
[ INFO] [1675498662.846276366]: Using planning interface 'CHOMP'
[ INFO] [1675498662.849389688]: Param 'default_workspace_bounds' was not set. Using default value: 10
[ INFO] [1675498662.850312737]: Param 'start_state_max_bounds_error' was set to 0.1
[ INFO] [1675498662.850965159]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1675498662.851453380]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1675498662.852524758]: Param 'jiggle_fraction' was set to 0.05
[ INFO] [1675498662.853169994]: Param 'max_sampling_attempts' was not set. Using default value: 100
[ INFO] [1675498662.853229042]: Using planning request adapter 'Limiting Max Cartesian Speed'
[ INFO] [1675498662.853247517]: Using planning request adapter 'Add Time Parameterization'
[ INFO] [1675498662.853255633]: Using planning request adapter 'Resolve constraint frames to robot links'
[ INFO] [1675498662.853263239]: Using planning request adapter 'Fix Workspace Bounds'
[ INFO] [1675498662.853285817]: Using planning request adapter 'Fix Start State Bounds'
[ INFO] [1675498662.853305943]: Using planning request adapter 'Fix Start State In Collision'
[ INFO] [1675498662.853318163]: Using planning request adapter 'Fix Start State Path Constraints'
[INFO] [1675498662.854790]: ROS Serial Python Node
[ INFO] [1675498662.854936749]: Loading planning pipeline 'ompl'
[INFO] [1675498662.861417]: Connecting to /dev/ttyUSB0 at 57600 baud
[ INFO] [1675498662.892744487]: Using planning interface 'OMPL'
[ INFO] [1675498662.894387413]: Param 'default_workspace_bounds' was not set. Using default value: 10
[ INFO] [1675498662.894826534]: Param 'start_state_max_bounds_error' was set to 0.1
[ INFO] [1675498662.895290065]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1675498662.895522821]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1675498662.895752335]: Param 'jiggle_fraction' was set to 0.05
[ INFO] [1675498662.895969859]: Param 'max_sampling_attempts' was not set. Using default value: 100
[ INFO] [1675498662.896008460]: Using planning request adapter 'Limiting Max Cartesian Speed'
[ INFO] [1675498662.896023823]: Using planning request adapter 'Add Time Parameterization'
[ INFO] [1675498662.896036012]: Using planning request adapter 'Resolve constraint frames to robot links'
[ INFO] [1675498662.896047842]: Using planning request adapter 'Fix Workspace Bounds'
[ INFO] [1675498662.896060102]: Using planning request adapter 'Fix Start State Bounds'
[ INFO] [1675498662.896071821]: Using planning request adapter 'Fix Start State In Collision'
[ INFO] [1675498662.896083200]: Using planning request adapter 'Fix Start State Path Constraints'
[ INFO] [1675498662.896126945]: Loading planning pipeline 'pilz_industrial_motion_planner'
[ INFO] [1675498662.899354430]: Reading limits from namespace /robot_description_planning
[ INFO] [1675498662.910253385]: Available plugins: pilz_industrial_motion_planner::PlanningContextLoaderCIRC pilz_industrial_motion_planner::PlanningContextLoaderLIN pilz_industrial_motion_planner::PlanningContextLoaderPTP 
[ INFO] [1675498662.910282178]: About to load: pilz_industrial_motion_planner::PlanningContextLoaderCIRC
[ INFO] [1675498662.911468338]: Registered Algorithm [CIRC]
[ INFO] [1675498662.911488254]: About to load: pilz_industrial_motion_planner::PlanningContextLoaderLIN
[ INFO] [1675498662.912595090]: Registered Algorithm [LIN]
[ INFO] [1675498662.912627486]: About to load: pilz_industrial_motion_planner::PlanningContextLoaderPTP
[ INFO] [1675498662.913739126]: Registered Algorithm [PTP]
[ INFO] [1675498662.913771542]: Using planning interface 'Pilz Industrial Motion Planner'
[INFO] [1675498662.939156]: Controller Spawner: Loaded controllers: joint_state_controller, arm_position_controller
[INFO] [1675498663.039109]: Started controllers: joint_state_controller, arm_position_controller
[ INFO] [1675498663.155436983]: Added FollowJointTrajectory controller for arm_position_controller
[ INFO] [1675498663.155730445]: Returned 1 controllers in list
[ INFO] [1675498663.167748075]: Trajectory execution is managing controllers
[ INFO] [1675498663.167800667]: MoveGroup debug mode is ON
Loading 'move_group/ApplyPlanningSceneService'...
Loading 'move_group/ClearOctomapService'...
Loading 'move_group/MoveGroupCartesianPathService'...
Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
Loading 'move_group/MoveGroupGetPlanningSceneService'...
Loading 'move_group/MoveGroupKinematicsService'...
Loading 'move_group/MoveGroupMoveAction'...
Loading 'move_group/MoveGroupPickPlaceAction'...
Loading 'move_group/MoveGroupPlanService'...
Loading 'move_group/MoveGroupQueryPlannersService'...
Loading 'move_group/MoveGroupStateValidationService'...
Loading 'pilz_industrial_motion_planner/MoveGroupSequenceAction'...
[ INFO] [1675498663.229225217]: initialize move group sequence action
[ INFO] [1675498663.238204637]: Reading limits from namespace /robot_description_planning
Loading 'pilz_industrial_motion_planner/MoveGroupSequenceService'...
[ INFO] [1675498663.246642028]: Reading limits from namespace /robot_description_planning
[ INFO] [1675498663.259115199]: 

********************************************************
* MoveGroup using: 
*     - ApplyPlanningSceneService
*     - ClearOctomapService
*     - CartesianPathService
*     - ExecuteTrajectoryAction
*     - GetPlanningSceneService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
*     - SequenceAction
*     - SequenceService
********************************************************

[ INFO] [1675498663.260111029]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1675498663.260162390]: MoveGroup context initialization complete

You can start planning now!

[INFO] [1675498664.978276]: Requesting topics...
[INFO] [1675498665.012896]: Note: subscribe buffer size is 280 bytes
[INFO] [1675498665.014946]: Setup subscriber on /lower_controller/command [std_msgs/Int32MultiArray]
[ INFO] [1675498666.085691962]: Loading robot model 'myarm_description'...
[ INFO] [1675498666.085780811]: No root/virtual joint specified in SRDF. Assuming fixed joint
[ INFO] [1675498666.313820762]: Starting planning scene monitor
[ INFO] [1675498666.316527534]: Listening to '/move_group/monitored_planning_scene'
[ INFO] [1675498666.389506903]: Constructing new MoveGroup connection for group 'arm' in namespace ''
[ INFO] [1675498667.361943745]: Ready to take commands for planning group arm.
```


* rostopic list

```bash
liyui@liyui:~$ rostopic list
/arm_position_controller/command
/arm_position_controller/follow_joint_trajectory/cancel
/arm_position_controller/follow_joint_trajectory/feedback
/arm_position_controller/follow_joint_trajectory/goal
/arm_position_controller/follow_joint_trajectory/result
/arm_position_controller/follow_joint_trajectory/status
/arm_position_controller/state
/attached_collision_object
/collision_object
/diagnostics
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
/joint_states
/lower_controller/command
/move_group/cancel
/move_group/display_contacts
/move_group/display_cost_sources
/move_group/display_grasp_markers
/move_group/display_planned_path
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/motion_plan_request
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_pipelines/ompl/ompl/parameter_descriptions
/move_group/planning_pipelines/ompl/ompl/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/recognized_object_array
/rosout
/rosout_agg
/rviz_liyui_8474_298277335083509303/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_liyui_8474_298277335083509303/motionplanning_planning_scene_monitor/parameter_updates
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full
/sequence_move_group/cancel
/sequence_move_group/feedback
/sequence_move_group/goal
/sequence_move_group/result
/sequence_move_group/status
/statistics
/tf
/tf_static
/trajectory_execution_event
```


* joint1 from 90 degrees to 0 degrees

```bash
liyui@liyui:~$ rostopic echo /arm_position_controller/follow_joint_trajectory/goal 

header: 
  seq: 2
  stamp: 
    secs: 145
    nsecs: 187000000
  frame_id: ''
goal_id: 
  stamp: 
    secs: 145
    nsecs: 187000000
  id: "/move_group-3-145.187000000"
goal: 
  trajectory: 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "world"
    joint_names: 
      - joint1
      - joint2
      - joint3
      - joint4
    points: 
      - 
        positions: [1.5700215958118209, -0.010944185320239441, -0.008365390433739073, -0.001859931908984258]
        velocities: [0.0, 0.0, 0.0, 0.0]
        accelerations: [-0.9965810905041843, 0.0, 0.0, 0.0]
        effort: []
        time_from_start: 
          secs: 0
          nsecs:         0
      - 
        positions: [1.449252886194951, -0.01064753652762003, -0.008144712115348621, -0.0018140150627876635]
        velocities: [-0.4155293966381329, 0.0010206807226940486, 0.0007592887990838678, 0.00015798628184507602]
        accelerations: [-0.9747801887395546, 0.0023943897966308765, 0.0017811969137850262, 0.0003706161318096062]
        effort: []
        time_from_start: 
          secs: 0
          nsecs: 492306862
      - 
        positions: [1.328484176578081, -0.010350887735000619, -0.007924033796958172, -0.0017680982165910687]
        velocities: [-0.6068734691780406, 0.0014906864739678513, 0.00110892810794164, 0.00023073622160445302]
        accelerations: [-0.21206806636789063, 0.0005209108885964296, 0.0003875078604288458, 8.06293022216974e-05]
        effort: []
        time_from_start: 
          secs: 0
          nsecs: 698485848
      - 
        positions: [1.2077154669612111, -0.010054238942381208, -0.0077033554785677204, -0.0017221813703944741]
        velocities: [-0.6280000000000001, 0.0015425803782784394, 0.00114753220754662, 0.00023876863057443364]
        accelerations: [0.0, 0.0, 0.0, 5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 0
          nsecs: 890792711
      - 
        positions: [1.0869467573443412, -0.009757590149761797, -0.007482677160177269, -0.0016762645241978793]
        velocities: [-0.6280000000000001, 0.0015425803782784394, 0.00114753220754662, 0.00023876863057443364]
        accelerations: [0.0, 0.0, 0.0, -5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 1
          nsecs:  83099573
      - 
        positions: [0.9661780477274713, -0.009460941357142386, -0.007261998841786818, -0.0016303476780012848]
        velocities: [-0.6280000000000001, 0.0015425803782784394, 0.0011475322075466178, 0.00023876863057443364]
        accelerations: [0.0, 0.0, -2.255150250361825e-17, 5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 1
          nsecs: 275406435
      - 
        positions: [0.8454093381106014, -0.009164292564522974, -0.007041320523396367, -0.00158443083180469]
        velocities: [-0.6280000000000001, 0.0015425803782784394, 0.0011475322075466157, 0.00023876863057443364]
        accelerations: [0.0, 0.0, 0.0, -5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 1
          nsecs: 467713298
      - 
        positions: [0.7246406284937315, -0.008867643771903563, -0.006820642205005917, -0.0015385139856080954]
        velocities: [-0.6280000000000001, 0.001542580378278444, 0.0011475322075466178, 0.00023876863057443364]
        accelerations: [0.0, 4.7358155257598326e-17, 2.255150250361825e-17, 5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 1
          nsecs: 660020160
      - 
        positions: [0.6038719188768615, -0.00857099497928415, -0.006599963886615465, -0.0014925971394115006]
        velocities: [-0.6280000000000001, 0.001542580378278444, 0.0011475322075466178, 0.00023876863057443364]
        accelerations: [0.0, -4.7358155257598326e-17, -2.255150250361825e-17, -5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 1
          nsecs: 852327023
      - 
        positions: [0.4831032092599916, -0.00827434618666474, -0.006379285568225015, -0.001446680293214906]
        velocities: [-0.6280000000000001, 0.0015425803782784394, 0.0011475322075466178, 0.00023876863057443364]
        accelerations: [0.0, 0.0, 2.255150250361825e-17, 5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 2
          nsecs:  44633885
      - 
        positions: [0.3623344996431217, -0.007977697394045328, -0.006158607249834564, -0.0014007634470183112]
        velocities: [-0.6280000000000001, 0.0015425803782784394, 0.0011475322075466178, 0.00023876863057443364]
        accelerations: [0.0, 0.0, -2.255150250361825e-17, -5.778822516552177e-18]
        effort: []
        time_from_start: 
          secs: 2
          nsecs: 236940748
      - 
        positions: [0.24156579002625178, -0.007681048601425917, -0.005937928931444113, -0.0013548466008217166]
        velocities: [-0.6068734691780406, 0.0014906864739678513, 0.001108928107941642, 0.00023073622160445302]
        accelerations: [0.21206806636789063, -0.0005209108885964296, -0.0003875078604287816, -8.06293022216974e-05]
        effort: []
        time_from_start: 
          secs: 2
          nsecs: 429247610
      - 
        positions: [0.12079708040938186, -0.007384399808806506, -0.005717250613053662, -0.0013089297546251218]
        velocities: [-0.41552939663813293, 0.0010206807226940486, 0.000759288799083871, 0.00015798628184507602]
        accelerations: [0.974780188739554, -0.002394389796630876, -0.0017811969137850555, -0.00037061613180960613]
        effort: []
        time_from_start: 
          secs: 2
          nsecs: 635426596
      - 
        positions: [2.8370792511850593e-05, -0.007087751016187095, -0.005496572294663211, -0.0012630129084285273]
        velocities: [0.0, 0.0, 0.0, 0.0]
        accelerations: [0.9965810905041845, -0.0024479401840367568, -0.0018210332783208268, -0.00037890493986633613]
        effort: []
        time_from_start: 
          secs: 3
          nsecs: 127733458
  path_tolerance: []
  goal_tolerance: []
  goal_time_tolerance: 
    secs: 0
    nsecs:         0
---
```
