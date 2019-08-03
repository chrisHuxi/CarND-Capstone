[![Udacity - Self-Driving CarNanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


# Self-Driving-Car Capstone Project

![image](https://github.com/chrisHuxi/CarND-Capstone/blob/master/imgs/udacity_self-driving_car_capstone_project_simulator.gif)

---
## Team: Who Needs Driver?:
|           | Name                     |    E-Mail                        |      GitHub                                     |
| --------- | -------------------------| -------------------------------- | :----------------------------------------------:|
| Team Lead | Xi Hu                    |    chris_huxi@163.com            |    [Xi](https://github.com/chrisHuxi)           |
|           | Yuanhui Li               |    viglyh@163.com                |    [Yuanhui](https://github.com/YHCodes)        |
|           | Zyuanhua                 |    zzyuanhua@163.com             |    [Zyuanhua](https://github.com/zzyuanhua)     |
|           | Maharshi Patel           |    patelmaharshi94@gmail.com     |    [Maharshi](https://github.com/maharshi3patel)|
|           | Aniket Satbhai           |    anks@tuta.io                  |    [Aniket](https://github.com/AnkS4)           |

## How to run test?
### 1. test on simulator
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
wait until we get log info: 
```
[INFO] [1564778567.666529]: loaded ssd detector!
```
then, open the simulator, check the "Camera" and uncheck the "Manual"
you will see the car runs like video [here](https://www.youtube.com/watch?v=53VO-UKxIKU&feature=youtu.be).


**optional**: if you want to show how the camera output looks like, you can run in another terminal:

```bash
rosrun rviz rviz
```
then add topic: image_color/raw.

**optional**: if you want to run 2nd test lot, you need to modify ros/src/waypoint_loader/launch/waypoint_loader.launch like this:

```html
#<param name="path" value="$(find styx)../../../data/wp_yaw_const.csv" />
<param name="path" value="$(find styx)../../../data/churchlot_with_cars.csv"/>       
<param name="velocity" value="5" />
```
then launch styx.launch and run simulator. you will see the car runs like video [here](https://www.youtube.com/watch?v=aBc22Gv5GgY&feature=youtu.be)

### 2. test on real world test bag:
Firstly download the [bag file](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM) 
then run in terminal :
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/site.launch
```
And wait until we get log info:
```
[INFO] [1564778567.666529]: loaded ssd detector!
```
then run in other terminal:
```bash
rosbag play -l XXX.bag
```

if you want to see the image given by bag file, open one another terminal and run:
```bash
rosrun rviz rviz
```
add topic: image_color/raw, you will see result as this [video: just_traffic_light.bag](https://www.youtube.com/watch?v=h7DuEj_1jVs&feature=youtu.be), [video: loop_with_traffic_light.bag](https://www.youtube.com/watch?v=AOS5C2_APOo&feature=youtu.be), [video: udacity_succesful_light_detection.bag](https://www.youtube.com/watch?v=nDF-dz21smM&feature=youtu.be)



## System Architecture

The project has following architecture:

![ROS Graph](./imgs/ros-graph.png)

### Waypointer Updater Node

Waypointer Updater node helps updating the target velocity of each waypoints based on traffic light and obstacle detection data.

- It subscribes to the /base_waypoints, /current_pose and /traffic_waypoint topics.
- It publishes a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

Subscribed Topics:
#### /base_waypoints
Msg Type: styx_msgs/Lane

This topic provides the waypoints along the driveway path. Waypoint Loader node publishes the list of waypoints to this topic at the starting.

#### /current_pose
Msg Type: geometry_msgs/PoseStamped

This topic provides the current position of the vehicle. The position is published by the Car/Simulator.

#### /traffic_waypoint
Msg Type: std_msgs/Int32

This topic provides the waypoints at which the car is expected to halt. Traffic Light Detection node publishes to this topic.

Published topics:
#### /final_waypoints
Msg Type: styx_msgs/Lane

Final waypoints are published to this topic. The vehicle is supposed to follow these waypoints.

### Traffic Light Detection Node

Traffic Light Detection Node detects the traffic light and publishes it's location.

- It subscribes to the /base_waypoints, /current_pose, /vehicle/traffic_lights and /image_raw topics.
- It publishes waypoint index of upcoming traffic light position to /traffic_waypoint.

Subscribed Topics:
#### /base_waypoints
Msg Type: styx_msgs/Lane

This topic provides the waypoints along the driveway path. These are the same list of waypoints used in Waypoint Updater node.

#### /current_pose
Msg Type: geometry_msgs/PoseStamped

This topic provides the current position of the vehicle. The position is published by the Car/Simulator.

#### /image_raw
Msg Type: sensor_msgs/Image

This topic provides raw image from the vehicle sensor. The image helps identify red lights in the incoming camera image.

#### /vehicle/traffic_lights
Msg Type: styx_msgs/TrafficLightArray

This topic is only used while using the simulator for testing the vehicle path without the use of classifier.
This topic provides the location of the traffic light in 3D map space and helps acquire an accurate ground truth data source for the traffic light classifier by sending the current color state of all traffic lights in the simulator.

Published Topics:
#### /traffic_waypoint
Msg Type: std_msgs/Int32

This topic provides the waypoints at which the car is expected to halt.


### DBW Node

Drive-By-Wire Node uses the final waypoints to apply required brake, steering and throttle values to drive the vehicle.

- It subscribes to the /twist_cmd, /vehicle/dbw_enabled and /current_velocity topics.
- It publishes /vehicle/brake_cmd, /vehicle/steering_cmd and /vehicle/throttle_cmd topics.

Subscribed Topics:
#### /twist_cmd
Msg Type: geometry_msgs/TwistStamped

This topic provides the proposed linear and angular velocities. Wapoint Follower Node publishes the message to this topic.

#### /vehicle/dbw_enabled
Msg Type: std_msgs/Bool

This topic indicates if the car is under dbw or driver control. In the simulator, it'll always be True. But, for the actual vehicle we have to make sure if dbw_enabled is True for driving autonomously.

#### /current_velocity
Msg Type: geometry_msgs/TwistStamped

This topic provides target linear and angular velocities the car should follow.

Published Topics:
#### /vehicle/brake_cmd
Msg Type: dbw_mkz_msgs/BrakeCmd

Required percent of throttle is published to this topic.

#### /vehicle/steering_cmd
Msg Type: dbw_mkz_msgs/SteeringCmd

Required steering angle is published to this topic.

#### /vehicle/throttle_cmd
Msg Type: dbw_mkz_msgs/ThrottleCmd

Required amount of torque is applied using this topic.


## About this project:
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Installation:

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
