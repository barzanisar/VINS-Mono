## Running VIMO

1. Edit "blackbird_config.yaml" with IMU, camera, thrust parameters such as noise standard deviations. 
You can also edit the parameter "output_path" to store results of the state, computation times and external force estimates in .csv format, which you can evaluate with matlab script "plot_force.m". 
"rpg_eval_path" stores pose estimates and groundtruth in the format required by rpg_trajectory_evaluation_toolbox

> Note: Initially you can set "record_sub_traj" to 0. When you set it to 1, the estimator only records trajectory estimates starting from "record_start_time" to "record_start_time" which are the start and end times of the trajectory.

Make sure you have already created the folders:

 1. <output_path>/<simulation_name> (to evaluate results with the matlab script "plot_force.m")

 2. <rpg_eval_path>/vimo/laptop_vimo_<simulation_name> (to evaluate VIMO pose estimates with rpg_trajectory_evaluation_toolbox)

 3. <rpg_eval_path>/vins/laptop_vins_<simulation_name> (to evaluate VINS pose estimates with rpg_trajectory_evaluation_toolbox)


If evaluating with rpg_evaluation_toolbox, make sure you create eval_cfg.yaml in folders 2 and 3 stated above. 

"eval_cfg.yaml" contents are:

```
align_type: posyaw

align_num_frames: -1  
``` 

2. Play the bag file containing the recorded dataset e.g.

```
rosbag play -s 22 picasso_maxSpeed1p0.bag picasso_maxSpeed1p0_images.bag
``` 

3. Let the bag play for 0.5-1 sec and then pause. This is to make sure the initial msgs are in correct sequence.

4. Launch VIMO.

```
roslaunch vins_estimator vimo.launch
``` 

5. Launch Rviz to visualize the estimated trajectory (in green).

```
roslaunch vins_estimator vins_rviz.launch
``` 

6. Unpause/Play the bag.

7. Once the trajectory has been tracked and you have applied forces, stop playing the bag and Ctrl-C the estimator and Rviz. 

> Note: VIMO fails when it receives zero control inputs so make sure you run VIMO when the bag is publishing thrust inputs on "control_topic".

8. To obtain trajectory top, side and box plots and overall trans and rot rmse, you will need to run Zichao's modified [trajectory evaluation toolbox](https://github.com/barzanisar/rpg_trajectory_evaluation/tree/rpg_master_thesis):

To generate vimo plots:

```
rosrun rpg_trajectory_evaluation analyze_trajectory_single.py /home/barza/catkin_ws/src/rpg_trajectory_evaluation/results/laptop/vimo/laptop_vimo_black_picasso_cY_1ms/ --recalculate_errors
``` 

To generate vins plots:

```
rosrun rpg_trajectory_evaluation analyze_trajectory_single.py /home/barza/catkin_ws/src/rpg_trajectory_evaluation/results/laptop/vins/laptop_vins_black_picasso_cY_1ms/ --recalculate_errors
```

To generate both plots against each other for the paper:

1. Edit "analyze_trajectories.py" with [simulation name](https://github.com/barzanisar/rpg_trajectory_evaluation/blob/rpg_master_thesis/scripts/analyze_trajectories.py#L49) you want to analyse and its trajectory's [max length](https://github.com/barzanisar/rpg_trajectory_evaluation/blob/rpg_master_thesis/scripts/analyze_trajectories.py#L63).

2. Run the python script to analyse VINS against VIMO:

```
rosrun rpg_trajectory_evaluation analyze_trajectories.py --platform laptop --output_dir /home/barza/catkin_ws/src/rpg_trajectory_evaluation/results --odometry_error --overall_odometry_error --plot_trajectories --rmse_table
``` 

