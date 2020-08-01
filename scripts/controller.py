#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import required Python code.
import os
import sys
import re
import socket
import threading
import pynput
from pynput.keyboard import Key, Listener
import time
import rospy
from std_msgs.msg import Float64

class RobotCarController(object):
    """Node example class."""

    def __init__(self, robot_host):
        self.robot_host = robot_host
        self.speedPub = rospy.Publisher(self.robot_host + '/control/speed', Float64, queue_size=10)
        self.steerPub = rospy.Publisher(self.robot_host + '/control/steer', Float64, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # Initialize message variables.
        self.enable = False
        self.message = ""

        self.w = False
        self.a = False
        self.s = False
        self.d = False
        self.q = False
        self.e = False

        self.speed = 0.0
        self.steer = 0.0

        print("w/s: accelerate")
        print("a/d: steer")
        print("q:   stops the engine")
        print("e:   neutral the servo")
        print("CTRL+C:   exit program")

        self.thread = threading.Thread(target=self.check_speedAndSteer, args=())
        self.thread.start()

        # Collect events until released
        with Listener(on_press=self.on_press, on_release=self.on_release,suppress=True) as listener:
            listener.join()

    def stop(self):
        """Turn off publisher."""
        self.enable = False
        self.speedPub.unregister()
        self.steerPub.unregister()
        self.thread.join()

    def printscreen(self):
        # The call os.system('clear') clears the screen.
        os.system('clear')
        print("w/s: forward / backward acceleration")
        print("a/d: left / right steering")
        print("q:   stop the motors")
        print("e:   neutral the motors")
        print("CTRL+C:   quit the program")
        print("========== Speed display ==========")
        print("Motor speed:  ", self.speed)

        self.message = "speed: %s and steer: %s" % (self.speed, self.steer)
        rospy.loginfo(self.message)

    def on_press(self, pressKey):
        # The robot car accelerates forward if the user presses the 
        # key "w" on the keyboard
        if pressKey.char == ('w'):
            self.w = True
            # The robot car accelerates in 10% increments 
            # each time the letter "w" is pressed, up to a maximum 
            # of 100 %. Then it'll go forward at maximum speed.
            self.speed = self.speed + 0.1

            if self.speed > 1:
                self.speed = 1

            # With the program L298NHBridge, which was imported at
            # the beginning of the program the rotating speed of the
            # left and right motors is set.
            # HBridge.setMotor(speed)
            self.printscreen()
        # By pressing the key "a" the robots cars turns to the left.
        elif pressKey.char == ('a'):
            self.a = True

            self.steer = self.steer - 0.05

            if self.steer < -1:
                self.steer = -1

            self.printscreen()
        # The robot car accelerates backwards if the user presses the 
        # key "s" on the keyboard
        elif pressKey.char == ('s'):
            self.s = True
            # The robot car brakes in 10% increments each time the letter
            # "s" is pressed until it stopps. If the user presses "s" once 
            # again the robot car accelerates backwards until a maximum 
            # speed of -100 %. Then it'll go backwards at maximum speed.
            self.speed = self.speed - 0.1

            if self.speed < -1:
                self.speed = -1

            # With the program L298NHBridge, which was imported at
            # the beginning of the program the rotating speed of the
            # left and right motors is set.
            #HBridge.setMotor(speed)
            self.printscreen()
        # By pressing the key "d" the robots cars turns to the right.
        elif pressKey.char == ('d'):
            self.d = True

            self.steer = self.steer + 0.05

            if self.steer > 1:
                self.steer = 1

            self.printscreen()
        elif pressKey.char == ('q'):
            self.q = True
            self.speed = 0
            #HBridge.setMotor(speed)
            self.printscreen()
        elif pressKey.char == ('e'):
            self.e = True
            self.steer = 0
            #HBridge.setMotor(speed)
            self.printscreen()
        elif pressKey.char == Key.ctrl:
            self.printscreen()
        elif pressKey.char == ('C'):
            self.printscreen()
        
    def on_release(self, releaseKey):
        self.w = False
        self.a = False
        self.s = False
        self.d = False
        self.q = False
        self.e = False

        if releaseKey.char == Key.ctrl:
            self.printscreen()
        elif releaseKey.char == ('C'):
            self.printscreen()

    def check_speedAndSteer(self):
        while ((self.w == False or self.s == False) or (self.a == False or self.d == False)) and not rospy.is_shutdown():
            if(self.speed >= 0.1):
                self.speed = self.speed - 0.1
                if(self.speed <= 0.1):
                    self.speed = 0.0

            if(self.speed <= -0.1):
                self.speed = self.speed + 0.1
                if(self.speed >= -0.1):
                    self.speed = 0.0
            
            speed_msg = Float64()
            speed_msg.data = self.speed

            self.speedPub.publish(speed_msg)

            if(self.steer >= 0.05):
                self.steer = self.steer - 0.05
                if(self.steer <= 0.05):
                    self.steer = 0.0

            if(self.steer <= -0.05):
                self.steer = self.steer + 0.05
                if(self.steer >= -0.05):
                    self.steer = 0.0
            
            self.printscreen()
            time.sleep(1)
            
            steer_msg = Float64()
            steer_msg.data = self.steer
            
            self.steerPub.publish(steer_msg)

            self.message = "speed: %s and steer: %s" % (self.speed, self.steer)

            rospy.loginfo(self.message)

            self.rate.sleep()

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_RobotCarController"
    rospy.init_node(node_name, anonymous=False)
    # Go to class functions that do all the heavy lifting.

    try:
        robotCarController = RobotCarController("robotcar")
    except rospy.ROSInterruptException:
        robotCarController.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
