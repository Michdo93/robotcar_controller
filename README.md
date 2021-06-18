# robotcar_controller

Is used on the control computer respectively operating computer to control the [RobotCar](https://github.com/Michdo93/robotcar). At first you have to make sure that the roscore is running and the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) is publishing informations.

![controller](https://raw.githubusercontent.com/Michdo93/robotcar_controller/master/controller.JPG)

## How to Use

As recommended in the [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials) the control computer respectively operating computer should run the roscore. So at first open a terminal window and execute `roscore`. After that open a second terminal window and run `rosrun robotcar_controller controller.py`.

The String variable `robot_host` uses the hostname of one RobotCar. As example it could be `robotcar`.

The following table shows the needed key's for controlling the RobotCar:

|                 Key                |            Function       |
|--------------------------------------------- | ------------------------------|
| <kbd>W</kbd> | forward acceleration or break if driving backwards |
| <kbd>A</kbd> | left steering or turning back to neutral if steering right |
| <kbd>S</kbd> | backward acceleration or break if driving forwards |
| <kbd>D</kbd> | right steering or turning back to neutral if steering left |
| <kbd>Q</kbd> | stops the motor / engine |
| <kbd>E</kbd> | turn the steering to neutral |
| <kbd>CTRL</kbd> + <kbd>C</kbd> | Quits the program |

## Publisher Informations

The Controller is publishing to the Control Node of the [RobotCar](https://github.com/Michdo93/robotcar). It uses the [pynput library](https://pynput.readthedocs.io/en/latest/) for key down and key up. So if a key is pressed the RobotCar will active change its behavious as the driver wants. If not key is pressed the Controller will slow down the speed and the steer turns slowly back to neutral. Means it will permanently published to Control Node.

|                 Topic Address                |            Message Type       |
|--------------------------------------------- | ------------------------------|
|robot_host + /control/speed                   | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)          |
|robot_host + /control/steer                   | [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)          |

An ADAS would intervene similarly in the driving process.
