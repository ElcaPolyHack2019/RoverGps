# RoverGps

# Prerequisites
* Install Docker Desktop for Windows
* Run the container `docker run --name ros -d -p 5901:5901 -p 6901:6901 -p 11311:11311 -p 9090:9090 henry2423/ros-vnc-ubuntu:melodic`

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
### Install the undistorted image node
`sudo apt-get install ros-melodic-image-proc`
### Install the apriltag library
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
See http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
1. `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0348 image:=/camera/image_raw camera:=/camera`
   * The small board is `0.0348`and the large is `0.076`.
1. Find the calibration data in `/tmp/calibrationdata.tar.gz`
1. Extract the calibration data
   1. `mkdir /tmp/calibrationdata/`
   1. `tar xvzf /tmp/calibrationdata.tar.gz -C /tmp/calibrationdata/`
1. Get the calibration file
   1. `cat /tmp/calibrationdata/ost.yaml`
1. Run the camera image capturing
1. View the undistorted image
   1. `ROS_NAMESPACE=camera rosrun image_proc image_proc`
