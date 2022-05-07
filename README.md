# Robotics 2021/2022 Project 1

- Team Members: Edoardo Fullin (10677606), Daniele Gazzola (10674966)

## Project folder stucture

- src: contains project source code
- srv: contains definition for the reset service
- msg: contains definition for custom messages (in this case Wheel.msg)
- launch: contains launch files for the project
- cfg: contains the python script to generate dynamic reconfigure headers for odometry

Note: Angles are always expressed in radians, except for the debug messages on the console where they are converted in degrees.

### Launch Files

We provide two launch files, project_nobag.launch launches without a bag (the bag must be run separately) while project_bag.launch also launches the rosbag play command and the bag specified in the bag argument (which must be present in bags folder and is not included in the submission). 
project_bag.launch was used for easier debugging and should not be used. 

Both launch files also contain the initial position of the robot (by default (0, 0, 0)) as well as the robot geometric parameters that, by default, have been tweaked a little to help match the ground thruth pose provided.

The default (not tweaked) parameters are also present but commented out, and it is possible to switch between the two without recompiling the source.

The name of the parameters are self-explainatory.

How to launch without a bag

```shell
$ roslaunch project1 project1_nobag.launch
```

### Sources and modules

#### vel_calc

Calculates the linear velocity of the robot by subscribing to to /wheel_states, publishes a TwistStamped message on /cmd_vel with the current speed.

#### odom

Computes odometry by subscribing to /cmd_vel and publishes a message of type Odometry on /odom 
Supports dynamic reconfigure and reset service (see below)

#### tf2_broadcaster

Broadcasts tf2 for the entire robot by subscribing to /odom.

#### wheel_calc

Publishes the needed angular speed for the wheels to have the desired linear speed.
Subscribes to /cmd_vel and publishes on /wheels_rpm

### Dynamic Reconfigure

It is possible to switch between the euler and Runge-Kutta iuntegration method by using dynamic reconfigure, the module is odom and the parameter name is 'odom_int', values are 'euler' for euler and 'rk' for runge-kutta.

```shell
$ rosrun dynamic_reconfigure dynparam set /odom odom_int rk # sets integration method to Runge-Kutta
```

### Services

We provide the 'Reset' service to set the current position, it requires 3 parameters for the new X, Y and Theta positions, all three must be of type float64 (double) and must be all specified.

```shell
$ rosservive call /odom/reset 0.0 0.0 3.14 # sets current position to (0,0) and current orientation to 180 degrees
```

### Messages

A custom message has been defined for the speed of the single wheels, as instructed by the specifications.

```
Header header
float64 rpm_fl
float64 rpm_fr
float64 rpm_rr
float64 rpm_rl
```

## TF

The tf broadcaster is launched by both launch files. 
It defines a 'world' reference system centered in (0.0) and a dynamic 'base_link' system following the robot (it subscribes to the /odom topic).

### Comments

The odometry follows pretty closely the ground thruth pose for bag1 and bag3.
The same is not true for bag2, especially for the Y coordinate.

We think the problem is related to the fact that the ground thruth orientation is not linear between 36 and 41 seconds but varies in bigger steps.
Also, it jumps from 175 degrees to -167 degrees which is not a 360 degrees difference, leaving a 18 degrees delta which our odometry does seem to not account for.

