# VIO_Utils

ROS Nodes to track performance parameters such as accuracy and CPU usage of VIO state-estimation algorithms

---
# Dependancies

Uses *cpu_monitor* to track CPU Usage
*cpu_monitor* installation can be found [here](https://github.com/alspitz/cpu_monitor)

---
# Usage

Run the below command to run the *vio_utils* node
Replace *estimated_state* and *ground_truth* with your respective ROS topics. Both should be publishing ROS Odometry messages
An Rviz terminal will open up in which green path is the 'Ground Truth' and red path is the 'State Estimation'
Add the nodes you want to monitor CPU usage for in the *source_list*. It will create a *csv* file with the CPU usage
Upon closing the node, the mean RMS error in *m* and *%* as well as the total distance traveled in *m* will be printed

```
#!command

roslaunch utils vio_utils.launch est_odom_topic:=estimated_state gt_topic:=ground_truth source_list:="[node_name1, node_name2]" 
```

