# Dynamically adjustable color filter package

### Overview
Basic Kinect-v1 (for the Xbox 360) node, with IPC support, based on [libfreenect](https://github.com/OpenKinect/libfreenect).
For now, it only supports a single Kinect device. (If multiple devices present, the first one listed by the `freenect_num_devices` will be selected). Use the

### Published topics
* `~image_raw` - RGB image(rgb8) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~camera_info` - RGB camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~depth/image_raw` - Depth camera image(mono16) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~depth/camera_info` - Depth camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~filtered_rgb` - RGB stream after filtering by color filter ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~filtered_circles` - RGB stream after shape filtering ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) 

## Instalation
### 1. Install libfreenect
The package was tested using a manual build from the [libfreenect](https://github.com/OpenKinect/libfreenect) github because the Kinect used, had a firmware version that requires specific build flags.

### 2. Copy the repo
Copy the repo to your workspace source folder.
~~~
cd ~/ws/src
git clone https://github.com/fadlio/kinect_ros2RGB
~~~

### 3. Install any missing ROS packages
Use `rosdep` from the top directory of your workspace to install any missing ROS related dependency.
~~~
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
~~~

### 4. Build your workspace
From the top directory of your workspace, use `colcon` to build your packages.
~~~
cd ~/ws
colcon build
~~~

### Usage
Use the following bash commands and the launch file to execute this package
1. Open 2 terminals and source the setup file in each terminal
~~~
source install/setup.bash
~~~
2. In one terminal execute the launch file showfilteredimage.launch.py
In the other terminal execute the color filter controller node
~~~
ros2 run kinect_ros2 kinect_color_filter_controller_node
~~~
~~~
ros2 run kinect_ros2 kinect_color_filter_controller_node
~~~

3. Specify the filter required to be used using the keys as comma seperated values. Eg: r1,r2,bl,
Note: Do not have spaces in the key names, spaces between the item and the comma.

