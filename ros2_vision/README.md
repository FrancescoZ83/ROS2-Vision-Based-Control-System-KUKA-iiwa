# ros2_vision

## :package: About

This package contains the tutorial code to create and run your C++ vision node using [usb_cam](https://github.com/ros-drivers/usb_cam), [camera_calibration](https://github.com/ros-perception/image_pipeline/tree/rolling/camera_calibration), [open_cv](https://github.com/ros-perception/vision_opencv), and [aruco_ros](https://github.com/pal-robotics/aruco_ros).

## :pencil: Before you start
Make sure to install the following packages with

```
$ sudo apt-get install ros-humble-usb-cam -y
```
```
$ sudo apt-get install ros-humble-image-pipeline -y
```
```
$ sudo apt-get install ros-humble-tf-transformations -y
```

To use the *usb_cam* package it is recommended to copy the default params file from the *usb_cam* default installation directory into your ROS2 workspace by
```
$ sudo cp /opt/ros/humble/share/usb_cam/config/params_1.yaml ~/ros2_ws/src/ros2_vision/config/camera_params.yaml 
```
To use the copied params file, you can use the command
```
$ ros2 run usb_cam usb_cam_node_exe --ros-args --params-file src/ros2_vision/config/camera_params.yaml 
```
NOTE: if you run into permission issues, remember to
```
$ sudo chown root:user /dev/video0 
```

At this point, you should be able to see the */image_raw* topic streamed by your device. To perform the calibration of your camera you must run the following node (more detailed instructions [here](https://docs.ros.org/en/rolling/p/camera_calibration/))
```
$ ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.015 --ros-args -r image:=/image_raw 
```
After calibration, it is recommended to copy the dataset into your preferred location, e.g.
```
mv /tmp/calibrationdata.tar.gz ~/ros2_ws/src/calibrationdata.tar.gz 
```
Unzip the *calibrationdata.tar.gz* file and copy the results of your calibration procedure
```
sudo cp /calibrationdata/ost.yaml /opt/ros/humble/share/usb_cam/config/my_camera_info.yaml 
```
Remember to modify the */ros2_vision/config/camera_params.yaml* file, to use the new file *my_camera_info.yaml*.

To rectify the image streamed by your camera using the results of your calibration you must run
```
$ ros2 run usb_cam usb_cam_node_exe --ros-args --params-file src/camera_params.yaml --remap image_raw:=image 
```
```
$ ros2 run image_proc rectify_node  
```
You should be able to see the */image_rect* topic.

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
$ rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package
```
$ colcon build --symlink-install
```
Source the setup files
```
$ . install/setup.bash
```

## :white_check_mark: Usage
Run the aruco_ros node
```
$ ros2 run usb_cam usb_cam_node_exe --ros-args -r /image_raw:=/stereo/left/image_rect_color -r /camera_info:=/stereo/left/camera_info
```
```
$ ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```