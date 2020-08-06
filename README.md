POSE_GRAPH_BACKEND
==================
Repository for multi-agent sensor fusion.

### Installation ###
Assuming you have a fully setup Ubuntu 16.04 distribution 
with a ROS-Kinetic installation and an initialized workspace, 
you need to install the following dependencies:

* catkin_simple :

        git clone https://github.com/catkin/catkin_simple.git

* comm_msgs : 
        
        git clone https://github.com/karrerm/comm_msgs.git
      
* eigen_catkin :

        git clone https://github.com/ethz-asl/eigen_catkin.git
        
* suitesparse :

        git clone https://github.com/ethz-asl/suitesparse
        
* glog_catkin :

        git clone https://github.com/ethz-asl/glog_catkin

* gflags_catkin :

        git clone https://github.com/ethz-asl/gflags_catkin
        
* ceres_catkin :

        git clone https://github.com/ethz-asl/ceres_catkin.git
        
* opencv3_catkin :

        git clone https://github.com/ethz-asl/opencv3_catkin.git
        
* ethzasl_brisk :

        git clone https://github.com/ethz-asl/ethzasl_brisk.git
        
* aslam_cv2 (here actually only aslam_cv and aslam_cameras are needed) :

        git clone https://github.com/ethz-asl/aslam_cv2.git
        
* geodetic_utils :

        git clone https://github.com/ethz-asl/geodetic_utils.git
        
* opengv :
   
        git clone https://github.com/ethz-asl/opengv.git
        
* robopt :

        git clone https://github.com/VIS4ROB-lab/robopt.git
        
* planner_msgs :
        
        git clone ???
       
# pose_graph_backend
Loop Closure backend for Collaborative Keyframe based VIO.

This was built as part of a Collaborative SLAM client-server framework consisting of the following software packages:
* Client (On Board UAV):
  * vins_client_server (multi-agent branch: https://github.com/VIS4ROB-lab/vins_client_server/tree/feature/multi_agent)
  * image_undistort (https://github.com/ethz-asl/image_undistort)
  * pcl_fusion (https://github.com/btearle/pcl_fusion)
  
* Server (Backend PC):
  * pose_graph_backend (multi-agent branch)
  * Voxblox (multi-agent fork: https://github.com/btearle/voxblox/tree/devel/multi-agent)
  * comm_msgs (multi-agent branch: https://github.com/karrerm/comm_msgs/tree/devel/multi-agent)
  
All the above must be installed to test the full system. This was developed and verified in ROS Kinetic on Ubuntu 16.04.

## Topics ##
The following are the ros topics the pose graph backend node subscribes to. The X represents the x-th agent:
* /keyframeX - custom keyframe message described in comm_msgs sent by pose graph node in vins_client_server
* /odometryX - current odometry estimate sent by vins_client_server
* /gpsX - GPS measurement directly from sensor
* /fused_pclX - custom pointcloud message described in comm_msgs sent by pcl fusion node
* /fake_gps2_X - leica laser measurement data used in Euroc dataset to simulate GPS messages

The following are published by pose graph backend:
* /pcl_transformX - pointcloud and its world frame pose, described in comm_msgs
* /pathX - RVIZ trajectory estimae
* /camera_pose_visualX - RVIZ camera visualization

## Parameters ##
* gps_align_num_corr - number of required GPS-odometry correspondences to start initial GPS alignment.
* gps_align_cov_max - the threshold that the covariance of the initial GPS alignment must be under for acceptance. Increase this is there is an issue with GPS initialization (system requires this for active gps agents)
* rel_pose_corr_min - minimum required number of correspondences following a loop detection relative pose optimization needed to accept the loop closure.
* rel_pose_outlier_norm_min - norm residual value that a residual of the loop detection relative pose optimization must be above in order to consider that residual an "outlier" and remove it from the following optimization
* local_opt_window_size - size of the window of recent keyframes for sliding window local pose graph optimization
* loop_image_min_matches - minimum keypoint matches in an image to continue with loop closure candidate
* loop_detect_sac_thresh - threshold for classifying a point as an inlier in RANSAC 3D-2D P3P in loop detection. Lower value is a stricter condition.
* loop_detect_sac_max_iter - max number of iterations for the RANSAC 3D-2D P3P in loop detection
* loop_detect_min_sac_inliers - minimum number of inliers of RANSAC 3D-2D P3P in loop detection to continue with loop closure candidate
* loop_detect_min_sac_inv_inliers - minimum number of inliers of the INVERSE RANSAC 3D-2D P3P in loop detection to continue with loop closure candidate
* loop_detect_min_pose_inliers - the final threshold of inliers from the relative pose graph optimization of the loop detection process to verify loop closure.
* loop_detect_reset_time - the amount of time following a loop closure where no new loop closures are looked for for that agent
* loop_detect_skip_kf - number of keyframes to skip loop detection for. (1 processes every keyframe, 2 would process every other keyframe, etc.)
* information_XXX - information values for the different residuals in pose graph optimization. Higher values indicate more certainty about the measurement.
* ignore_gps_altitude - set to TRUE typically if running in a live setup due to large fluctuations in GPS altitude. Uses the odometry altitude value instead.
* gps_active_X - whether agent X has an active GPS.
* gps_referenceX - the reference point for local GPS coordinates.
* gps_offsetX - the translational position offset between IMU and GPS antenna onboard UAV.

## General setup ##
The following describes how to run the system. A VINS-Mono yaml configuration file is required for the vins_client_server package.
A stereo yaml configuration file is required for the dense_stereo package from image_undistort, and an ncamera yaml configuration file is required for
the pcl_fusion package. Examples of these can be found in the conf folder.

On the PC, run the following in separate terminals:
``` 
  roslaunch pose_graph_backend pose_graph_node_v4rl.launch
  roslaunch voxblox_ros v4rl.launch
```
Wait for the "Read parameters and started threads" message to appear in terminal. The pose_graph_backend should launch RVIZ.
On the UAV, run the following in separate terminals. The launch files must be edited to reflect assigned UAV ids
in the agent_id parameter:
```
  roslaunch vins_estimator v4rl.launch
  roslaunch pcl_fusion pcl_fusion_node_v4rl.launch
```
The pcl_fusion should launch dense_stereo from image_undistort. The UAV should now be able to begin flying, and the agent's
trajectory should appear in RVIZ after the world frame initialization has completed in the pose_graph_backend.

## Euroc Dataset ##
### Single Agent ###
The following shows how to run a single agent of the euroc dataset with a simulation of the full pipeline on your PC.
(https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
(using the Leica position measurements as a fake GPS).

Launch the backend software in separate terminals:
```
        roslaunch pose_graph_backend pose_graph_node_euroc.launch
        roslaunch voxblox_ros euroc_dataset.launch
```
Wait for the "Read parameters and started threads" message to appear for the pose_graph_backend.

Launch the client software in separate terminals (make sure the agent_id parameters match in the launch files):
```
  roslaunch vins_estimator multi_euroc_0.launch
  roslaunch pcl_fusion pcl_fusion_node_euroc.launch
```

Finally, launch the Euroc data:
```
  roslaunch multi_agent_play_0.launch
```

### Multi Agent ###
Running the full pipeline with more than one agent is not recommended on a single PC unless you have a powerful PC (since it would be running visual-inertial odometry, dense stereo pointcloud construction, pointcloud filtering, pose graph optimization, loop detection, and mesh reconstruction for every agent...). A better alternative
is to run the client-side software and record its output from the Euroc data in a bag file for different Euroc trajectories individually, and then play those bag files simultaneously while running the backend packages. This more closely reflects
the actual load on your pc. An example of the client software output from the first three Euroc datasets can be found in the data folder 
of this repository as prerecorded bag files. To simulate the multi-agent system:

Launch the backend software in separate terminals:
```
  roslaunch pose_graph_backend pose_graph_node_euroc.launch
  roslaunch voxblox_ros euroc_dataset.launch
```
Play each bag file in separate terminals:
```
  rosbag play MH_01_PreRecordedUAV.bag
  rosbag play MH_02_PreRecordedUAV.bag
  rosbag play MH_03_PreRecordedUAV.bag
  

