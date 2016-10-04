# README #

### What is this repository for? ###

This repository provides a simple contact detection. Based on the wrench measurement of a force sensor, the component is in charge of detecting any significant variation of the signal, which we relate at that level to an interaction taking place. 

### How do I get set up? ###

This repository is a python ROS package, so that the installation is straightforward according to ROS methodology:
```
# assuming we are at a root of a catkin workspace
git clone https://bitbucket.org/aremazeilles/contact_detection.git src/contact_detection
catkin build
```
So far, the single node defined is launched through the following command:
```
# assuming a ROS WrenchStamped is published on topic /wrench_stamped
rosrun contact_detection optoforce_contact_node.py wrench:=/wrench_stamped
```
To see whether a contact is being detected, one can use **to be done**

* How to run tests: **to be **done