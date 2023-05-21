#!/usr/bin/env python3
"""
Script to move Robot
"""
import rospy
from geometry_msgs.msg import Twist
import time


def move_robot():
    rospy.init_node('robot_publisher', anonymous=True)
    # Create a publisher which can "talk" to Robot and tell it to move
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set publish rate at 10 Hz
    rate = rospy.Rate(10)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()

    ######## Move Straight ########
    print("Moving Straight")
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 18:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    ######## Rotating Counterclockwise ########
    print("Rotating")
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.1            # rotate at 0.2 rad/sec

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 2:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
    
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 2:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
        
    
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = -0.2           # rotate at 0.2 rad/sec

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 2:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
        
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 7:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
        
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = -0.1           # rotate at 0.2 rad/sec

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 1:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
        
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 6:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
        
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.1          # rotate at 0.2 rad/sec

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 1:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
        
        
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    # For the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 5:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
    
   

    ######## Stop ########
    print("Stopping")
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0            # Giving both zero will stop the robot

    now = time.time()
    # For the next 1 seconds publish cmd_vel move commands
    while time.time() - now < 1:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    print("Exit")


if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
