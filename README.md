### Recorded Video, bad quality to avoid putting more lag to the system

[![Simulator Version](https://img.youtube.com/vi/5fF8uMRlYpU/0.jpg)](https://www.youtube.com/watch?v=5fF8uMRlYpU)

[![Site Version](https://img.youtube.com/vi/nYvcvkAD_l4/0.jpg)](https://www.youtube.com/watch?v=nYvcvkAD_l4)

### Some Words

Did this on my own, Not I am not teamworking, my last team leader disappeared!!! Left me with 2 weeks and an empty deposit.

Tested it and tuned the parameters on VirualtBox. The biggest challenge is handling Inference introduced lag with my poor compute power. Some lag compensention algorithms can work on others' envrioments / platforms, some might not. For example, have to use very low Rospy.rate, this leads to sampling lag and even light signals loss. Another concern is PID parameters for Brake which might also be affected by individual system.

1) Tested on rosbag, please wait for the project launch completion before launch rosbag. The recorded video shows large lag.  The command to show the rosbag camera: " rosrun image_view image_view image:=/image_color "

2) The car can handle wrapping starting point problem, even there's some gap between Waypoints start and end, which presented by Udacity

PS. The reason car doesn't stop precisly is due to my gear lag, the instability forced me to leave some margin when plan the stop.  Also the margin would lead the car to miss the lights change if it happens too close.  If had a GPU gear, you might try to increase the rospy.Rate, shorten the slowdown dist, tune the PID parameters. It shall stop precisly with these.  But the attempt to handle system lag was also interesting, at least for me :)

### End of Some Words


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

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
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
