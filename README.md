# mbplanner_ros

## Install and run:
- Follow the instructions in [mbplanner_ws](https://github.com/MihirDharmadhikari/mbplanner_ws) for installation.
- Launch the planner:
```bash
roslaunch mbplanner mbplanner_m100_sim.launch
```
This will launch the planner with the rviz visualizer with the UI which can be used to trigger the planner.  
Alternately, the planner can be triggered by calling the respective rosservices as follows:  
Start planner: ```rosservice call /planner_control_interface/std_srvs/automatic_planning "{}" ```  
Stop planner: ```rosservice call /planner_control_interface/std_srvs/stop ```

- We provide 4 different gazebo simulation environments which can be found in the planner_gazebo_sim package. The world file for the desired environment needs to be specified in the mbplanner_m100_sim.launch file here:
```xml
<arg name="world_file" default="$(find planner_gazebo_sim)/worlds/pittsburgh_mine.world"/>
```  
We also support the environments published by DARPA for the Subterranean Challenge - Cave circuit. In order to use those environments, the models and worlds need to be downloaded that have not been included in this repository. The package for the same can be downloaded from [here](https://github.com/MihirDharmadhikari/cave_model).
Compile this package: 
```bash
catkin build cave_models
```
Use the ```mbplanner/launch/mbplanner_m100_sim_cave.launch``` file:
```bash
roslaunch mbplanner mbplanner_m100_sim_cave.launch
```

### Select the mapping framework
The planner supports two mapping frameworks: Voxblox and Octomap
#### Voxblox
By default, the planner is compiled with Voxblox. Voxblox can be used with two mapping modes, Euclidean Signed Distance Fields (ESDF) and Truncated Signed Distance Fields (TSDF). By default TSDF is used in which the distance(to the closest occiped voxel) of a voxel is truncated to a fixed value.  
In order to switch to ESDF make the following changes:  
In the file ``` planner_common/include/planner_common/map_manager_voxblox_impl.h``` comment the following line:  
```C++
#define TSDF
```  
In the file ```mbplanner/launch/mbplanner_m100_sim.launch``` comment the following line:  
```xml
<remap from="global_planner_node/tsdf_map_in" to="mbplanner_node/tsdf_map_out"/>
```  
Uncomment the following line:
```xml
<!-- <remap from="global_planner_node/esdf_map_in" to="mbplanner_node/esdf_map_out"/> -->
```  
#### Octomap
To compile with Octomap, set the flag USE_OCTOMAP to 1:
```
catkin build -DCMAKE_BUILD_TYPE=Release -DUSE_OCTOMAP=1 mbplanner
```
Also change the config to Octomap in the ```mbplanner/launch/mbplanner_m100_sim.launch``` file
```xml
<arg name="map_config_file" default="$(arg octomap_config_file)"/>
```

## Formatting:
Whenever a change is made format the code using clang-format with the .clang-format file provided
Format all the .cpp, .hpp, .c, .h files
```bash
find . -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -style=file -i {} \;
```

## Tutorial:
You could find a short tutorial on the plannning algorithm and the overall architecture on our website: [Link](https://www.autonomousrobotslab.com/subtplanning.html)

## Reference:
If you use this work in your research, please cite the following publication.
```
@inproceedings{mbplanner2020,
  title={Motion Primitives-based Agile Exploration Path Planning for Aerial Robotics},
  author={Dharmadhikari, Mihir and Dang, Tung and Solanka, Lukas and Loje, Johannes and Nguyen, Huan and Khedekar, Nikhil and Alexis, Kostas},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2020},
  organization={IEEE}
}
```
