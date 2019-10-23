# RoverGps

# Prerequisites
* Install Docker Desktop for Windows
* Run the container `docker run --name ros -d -p 5901:5901 -p 6901:6901 -p 9090:9090 henry2423/ros-vnc-ubuntu:melodic`

# Installation and Configuration
### Update the ROS Package Repository GPG Key
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
```
### Update the apt-get cache
```sudo apt-get update```
### Install image view
`sudo apt-get install ros-melodic-image-view`
### Install the camera calibration package
`sudo apt-get install ros-melodic-camera-calibration`
### Install the apriltag library
`apt-get install ros-melodic-apriltag # Is this needed?`<br/>
`sudo apt-get install ros-melodic-apriltag-ros`
### Install the ROS bridge
`sudo apt-get install ros-melodic-rosbridge-server`
### Install the video stream package
`sudo apt-get install ros-melodic-video-stream-opencv`
### Test the video stream
`rosrun video_stream_opencv test_video_resource.py rtsp://viewer:viewer@192.168.1.50/axis-media/media.amp`

# Running
### Run the ROS master
`roscore`
### Run the ROS bridge
`roslaunch rosbridge_server rosbridge_websocket.launch`
### Run analysis tools
`rosrun rqt_graph rqt_graph`<br/>
`rosrun rqt_image_view rqt_image_view`<br/>
`rosrun rviz rviz`
### Run the image capturing
```
cd /home/ros/Desktop/gps/
roslaunch mycamera.launch
```
### Run the continuous detection
```
cd /home/ros/Desktop/gps/
roslaunch my_continuous_detection.launch
```

# One-time camera calibration
See http://library.isr.ist.utl.pt/docs/roswiki/camera_calibration(2f)Tutorials(2f)MonocularCalibration.html
1. `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0348 image:=/camera/image_raw camera:=/camera`
1. Find the calibration data in `/tmp/calibrationdata.tar.gz`
1. Extract the calibration data
   1. `mkdir /tmp/calibrationdata/`
   1. `tar xvzf /tmp/calibrationdata.tar.gz -C /tmp/calibrationdata/`

# File contents
### mycamera.launch
```
<launch>
   <!-- launch video stream -->
   <include file="$(find video_stream_opencv)/launch/camera.launch" >
        <!-- node name and ros graph name -->
        <arg name="camera_name" value="camera" />
        <!-- means video device 0, /dev/video0 -->
        <arg name="video_stream_provider" value="rtsp://viewer:viewer@192.168.1.50/axis-media/media.amp" />
        <!-- set camera fps to (if the device allows) -->
        <arg name="set_camera_fps" value="30"/>
        <!-- set buffer queue size of frame capturing to -->
        <arg name="buffer_queue_size" value="100" />
        <!-- throttling the querying of frames to -->
        <arg name="fps" value="30" />
        <!-- setting frame_id -->
        <arg name="frame_id" value="camera" />
        <!-- camera info loading, take care as it needs the "file:///" at the start , e.g.:
        "file:///$(find your_camera_package)/config/your_camera.yaml" -->
        <arg name="camera_info_url" value="file:///home/ros/Desktop/gps/camera_info.yaml" />
        <!-- flip the image horizontally (mirror it) -->
        <arg name="flip_horizontal" value="false" />
        <!-- flip the image vertically -->
        <arg name="flip_vertical" value="false" />
        <!-- visualize on an image_view window the stream generated -->
        <arg name="visualize" value="false" />
   </include>
</launch>
```
### camera_info.yaml
```
image_width: 1920
image_height: 1080
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [979.532873, 0.000000, 960.484420, 0.000000, 982.778415, 528.807332, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.278434, 0.061694, 0.001579, 0.002266, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [682.614807, 0.000000, 971.459725, 0.000000, 0.000000, 890.354187, 528.307678, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```
### my_continuous_detection.launch
```
<launch>
  <arg name="launch_prefix" default="" /> <!-- set to value="gdbserver localhost:10000" for remote debugging -->
  <arg name="node_namespace" default="apriltag_ros_continuous_node" />
  <arg name="camera_name" default="camera" />
  <arg name="camera_frame" default="camera" />
  <arg name="image_topic" default="image_raw" />

  <!-- Set parameters -->
  <rosparam command="load" file="/home/ros/Desktop/gps/config/settings.yaml" ns="$(arg node_namespace)" />
  <rosparam command="load" file="/home/ros/Desktop/gps/config/tags.yaml" ns="$(arg node_namespace)" />
  
  <node pkg="apriltag_ros" type="apriltag_ros_continuous_node" name="$(arg node_namespace)" clear_params="true" output="screen" launch-prefix="$(arg launch_prefix)" >
    <!-- Remap topics from those used in code to those on the ROS network -->
    <remap from="image_rect" to="$(arg camera_name)/$(arg image_topic)" />
    <remap from="camera_info" to="$(arg camera_name)/camera_info" />

    <param name="camera_frame" type="str" value="$(arg camera_frame)" />
    <param name="publish_tag_detections_image" type="bool" value="true" />      <!-- default: false -->
  </node>
</launch>
```
### config/settings.yaml
```
tag_family:        'tag36h11'
tag_border:        1 
tag_threads:       2 
tag_decimate:      1.0 
tag_blur:          0.0 
tag_refine_edges:  1 
tag_refine_decode: 0 
tag_refine_pose:   0 
tag_debug:         0 
publish_tf:        true
publish_tag_detections_image: true
```
### config/tags.yaml
```
standalone_tags:
  [
    {id: 50, size: 0.10},
    {id: 51, size: 0.10},
    {id: 52, size: 0.10},
    {id: 53, size: 0.10}
  ]
```
