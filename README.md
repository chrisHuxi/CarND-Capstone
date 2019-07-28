
## Deadline to submit: 10th, August


## Our Team: Who Needs Driver?
|           | Name                     |    E-Mail                        |      GitHub                                     |
| --------- | -------------------------| -------------------------------- | :----------------------------------------------:|
| Team Lead | Xi Hu                    |    Xi.Hu@mailbox.tu-dresden.de   |    [Xi](https://github.com/chrisHuxi)           |
|           | Yuanhui Li               |    viglyh@163.com                |    [Yuanhui](https://github.com/YHCodes)      |
|           | Zyuanhua                 |    zzyuanhua@163.com             |    [Zyuanhua](https://github.com/zzyuanhua)       |
|           | Maharshi Patel           |    patelmaharshi94@gmail.com     |    [Maharshi](https://github.com/maharshi3patel)|
|           | Aniket Satbhai           |    anks@tuta.io                  |    [Aniket](https://github.com/AnkS4)           |

## System Architecture

The project has following architecture:

![ROS Graph](./imgs/ros-graph.png)


## Our process so far: 

* **Simulation part:** almost done except a traffic light detector/classifier. Basically based on udacity-walkthrough. Some optimization was done to reduce latency. Passed on high way test scene.
* **Real-world test:** I think we can reuse code of simulator part, and use a another traffic light detector/classifier based on real-world images.
* **Not tested** on real-world data and second test slot scene yet. ==> **done**, everything gose well
* And I have got already some **[labeled data](https://drive.google.com/open?id=1ygWBMW8PYeXUSP2nCAwhTDXeU87SFkpV)** both from simulator and real-world. And Here is a **[article using tensorflow to train and use rcnn and ssd model](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62)**

## TODO-List:

* **Training** a traffic light detector/classifier for simulator, testing it on high way scene.
* **Training** a traffic light detector/classifier for real-world, testing it on [bag file](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip).
* Trying to **test code** on native ubuntu 16.04, see if the latency is acceptable or not. If not trying to optimize it. ==> **done**, latency issue improved significantly. 
* **Reviewing** whole project and writing README document.


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
