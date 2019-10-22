# RoverGps

# Installation
## Install image view
`apt-get install ros-melodic-image-view`
# Install the camera calibration package
`apt-get install ros-melodic-camera-calibration`
## Install the apriltag library
`apt-get install ros-melodic-apriltag # Is this needed?`<br/>
`apt-get install ros-melodic-apriltag-ros`
## Install the ROS bridge
`apt-get install ros-melodic-rosbridge-server`
## Install the video stream package
`apt-get install ros-melodic-video-stream-opencv`
### Test the video stream
`rosrun video_stream_opencv test_video_resource.py rtsp://viewer:viewer@192.168.1.50/axis-media/media.amp`

# Running
## Run the ROS master
`roscore`
## Run the ROS bridge
`roslaunch rosbridge_server rosbridge_websocket.launch`
## Run analysis tools
`rosrun rqt_graph rqt_graph`<br/>
`rosrun rqt_image_view rqt_image_view`<br/>
`rosrun rviz rviz`
## Run the image capturing
```
cd /home/ros/Desktop/tests/
roslaunch mycamera.launch
```

## launch the detection
```
cd /home/ros/Desktop/tests/
roslaunch my_continuous_detection.launch
```

# One-time camera calibration
See http://library.isr.ist.utl.pt/docs/roswiki/camera_calibration(2f)Tutorials(2f)MonocularCalibration.html
1. `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0348 image:=/camera/image_raw camera:=/camera`
2. Find the calibration data in `/tmp/calibrationdata.tar.gz`
3. Extract the calibration data
4. `mkdir /tmp/calibrationdata/`
5. `tar xvzf /tmp/calibrationdata.tar.gz -C /tmp/calibrationdata/`
