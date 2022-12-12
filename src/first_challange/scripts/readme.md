# VU Introduction to Robotic

## Explaination

### Nodes
franka_node.py: Node that controlls the Franka Emika Panda robotic arm.
We chose the usage of a node because of its simplicity and its contribution to maintain clean code

### Launch files
first_assignment.launch: This file is used to start the aformentioned node (franka_node.py) and thus
the execution of the algorithmn for the inverse kinematics calculation. It is the main starting point.

### Modification to scene
First we disabled gravity as we found it distracting that the robot arm crashes to the ground in the
begining. We also wanted to avoid interfering with the calculations, as we heard from other groups.

We first tried running the robot in velocity mode, however we encountered problems. Therefore we
decided to set the robot to position mode. With our first approach velocity mode was far more dificult
to compute. We found it easier to work with position mode.

### How to use it
Copy the folder into `catkin_ws/src`

```
cd ~/catkin_ws
catkin_make
```
then run

```
roslaunch first_challange first_assignment.launch
```



