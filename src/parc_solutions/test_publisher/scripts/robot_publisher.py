#!/usr/bin/env python3
"""
Script to move Robot
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, LaserScan, CameraInfo,Imu
from parc_robot.gps2cartesian import gps_to_cartesian
import time
import math


def calc_distance(coordA, coordB):
    pointA = [6.471, -2.124]
    pointB = [coordA, coordB]
    return math.dist(pointA, pointB)


ros_topics = {
    "gps/fix":{
        "data":None,
        "type":NavSatFix
    },
    "scan":{
        "data":None,
        "type":LaserScan
    },
    "left_camera/camera_info":{
        "data":None,
        "type":CameraInfo
    },
    "right_camera/camera_info":{
        "data":None,
        "type":CameraInfo
    },
    "zed2/imu/data":{
        "data":None,
        "type":Imu
    },
    
}


    


def shutdown_callback():
    rospy.loginfo("Shutting down...")
    rospy.signal_shutdown("Shutdown requested")


def start_robot_movement():
    # rate = rospy.Rate(10)
    # Create a Twist message with desired linear and angular velocities
    move_cmd = Twist()
    move_cmd.linear.x = 0.3  # Adjust the linear velocity as desired
    move_cmd.angular.z = 0.0  # Adjust the angular velocity as desired

    # Publish the movement command to the '/cmd_vel' topic
    pub.publish(move_cmd)
    
def stop_robot():
    # Create a Twist message with zero linear and angular velocities
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0

    # Publish the stop command to the '/cmd_vel' topic
    pub.publish(stop_cmd)   


def change_robot_direction(x,y):
    # rate = rospy.Rate(10)
    # Create a Twist message with desired linear and angular velocities
    move_cmd = Twist()
    move_cmd.linear.x = x # Adjust the linear velocity as desired
    move_cmd.angular.z = y  # Adjust the angular velocity as desired

    # Publish the movement command to the '/cmd_vel' topic
    pub.publish(move_cmd)
# def move_robot(gps):
def move_robot(data, args):
    

    # Create a Twist message and add linear x and angular z values
    # move_cmd = Twist()
    topic_data = ros_topics[args]["data"]
    topic_data = data
    gps = None
    
    
    if args == "scan":
        scan_data = data
        # print(scan_data)
        # Access the laser scan data
        ranges = scan_data.ranges  # List of range measurements
        min_range = min(ranges)  # Minimum range value
        # print(min_range)
        # Set the desired minimum safe distance to avoid obstacles
        safe_distance = 0.17  # Adjust this value according to your requirements

        if min_range < safe_distance:
            # Obstacle detected, stop the robot
            stop_robot()
        else:
            start_robot_movement()
    if args == "gps/fix":
        gps = topic_data

        x, y = gps_to_cartesian(gps.latitude, gps.longitude) # get the cartesian coordinates from the GPS coordinates.
        z = calc_distance(x,y)
        # itz_turned = False
        
        print(z)
        print("x" + str(x))
        print("y"+str(y))
        
        
    

        if z >= 3.956 and z < 4.0 and y>-3.0 and y<-2:
            print("this 1")
            stop_robot()
            change_robot_direction(0.0,0.3)
            time.sleep(2)
            start_robot_movement()
        elif z >= 4.0 and z < 5.1 and y>-3.0 and y<-2:
            print("this two")
            change_robot_direction(0.0,-0.457)
        elif z >= 10.0 and z < 10.2 and y>-2.90 and y < -2.91:
            print("this two two")
            change_robot_direction(0.0,0.1)
        elif z > 10.4 and x < -4.7 and y < -2.8:
            print("turning")
            change_robot_direction(0.0,-0.1)
            # stop_robot()
           
   
    
    # elif z > 11.0 and y>-3.5 and y<-2:
    #     print("passe 1 4")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.2
        
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 6:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z > 11  and x<-4.5  and y < -3.00:
    #     print("pase 1 5")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.123
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 4:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
        
    # elif z >= 10.37 and z < 11 and x <-3.4 and y < -3.8:
    #     print("obstacle right")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 4:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z >= 10.37 and z < 11 and x <-3.4 and y < -3.8  and not x < -4.1 and not y < -4:
    #     print("obstacle right")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.5
    #     now = time.time()
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 6:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     print("obstacle right")
    #     # move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     # move_cmd.angular.z = -0.2
    #     # now = time.time()
    #     # while time.time() - now < 1:
    #     #     pub.publish(move_cmd)           # publish to Robot
    #     #     rate.sleep()
    #     #     break
    #     # move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     # move_cmd.angular.z = 0.0
    #     # now = time.time()
    #     # # For the next 3 seconds publish cmd_vel move commands
    #     # while time.time() - now < 2:
    #     #     pub.publish(move_cmd)           # publish to Robot
    #     #     rate.sleep()
    #     #     break
    # elif z <= 9.244 and z > 8.6 and x >-2.5 and y < -4.3 :
    #     print("unexpected")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.1
    #     now = time.time()
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z <= 9.5 and z > 9.0 and x >-2.8 and y > -4.3  and not x > - 2.7 and not y > -2.7:
    #     print("obstacle right right")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.1
    #     now = time.time()
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    
    # elif z >= 10.68 and z < 11.1 and x <-4 and y < -4:
    #     print("obstacle")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.14
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     print("obstacle")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.1
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z < 10.5 and z >= 10 and  x > -3.3 and y > -3.9 :
    #     print("little slant 1")
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z < 7.3 and z >= 7.122 and  x > -0.5 and y > -3.8 :
    #     print("little slant")
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z < 7.9 and z <= 6.3 and  x > -0.3 and y > -4.5 and y < -3.0 and not x > 0.3 :
    #     print("obstacle 2 on second lane")
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.2
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z < 7.9 and z >= 5.9 and  x > -0.3 and y > -4.5 and y < -3.0 and not x > 0.3 :
    #     print("obastacle c on second lane 2")
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.15
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.2
    #     now = time.time()
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    
    # elif z < 6.5 and z >= 5.228  and  x < 1.1 and x > 0.2 and y > -4.0 and not y > -3.0 :
    #     print("obastacle c on second lane 3")
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.2
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 5:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # elif z < 5.2 and z <= 4.0 and  x > 0.3 and y > -4.5 and y < -3.0 and not x > 2.8:
    #     print("obastacle c on second lane")
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.2
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.0            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.2
    #     now = time.time()
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     while time.time() - now < 3:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
        
   
    
    # elif z < 3.6  and z > 3.2 and x>3.5  and y < -3.00:
    #     print("sigma 1")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     # print("here again")
    #     # move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     # move_cmd.angular.z = 1.0
    #     # now = time.time()
    #     # # For the next 3 seconds publish cmd_vel move commands
    #     # while time.time() - now < 1:
    #     #     pub.publish(move_cmd)           # publish to Robot
    #     #     rate.sleep()
    #     #     break
    #     # move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     # move_cmd.angular.z = 0.0
    #     # now = time.time()
    #     # # For the next 3 seconds publish cmd_vel move commands
    #     # while time.time() - now < 3:
    #     #     pub.publish(move_cmd)           # publish to Robot
    #     #     rate.sleep()
    #     #     break
    # elif z <= 3.5  and z > 3.2 and x<3.5  and y < -3.50:
    #     print("sigma2")
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.1
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 2:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.0             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = -0.1
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 1:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    #     move_cmd.linear.x = 0.3            # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
    #     now = time.time()
    #     # For the next 3 seconds publish cmd_vel move commands
    #     while time.time() - now < 5:
    #         pub.publish(move_cmd)           # publish to Robot
    #         rate.sleep()
    #         break
    # else:
    #     move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    #     move_cmd.angular.z = 0.0
        
    
    # ######## Move Straight ########
    # print("Moving Straight")
    # pub.publish(move_cmd)
    # print(z)
    # rospy.loginfo("The translation from the origin (0,0) to the gps location provided is {:.3f}, {:.3f} m. which gives us a distance of {:.3f}".format(x, y, z))

    


if __name__ == '__main__':
    try:
        rospy.init_node('robot_publisher', anonymous=True)
        # Create a publisher which can "talk" to Robot and tell it to move
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        time.sleep(1)
        # rate = rospy.Rate(10)
        start_robot_movement()
        for key in ros_topics.keys():
            rospy.Subscriber(name=key, data_class= ros_topics[key]["type"], callback=move_robot, callback_args=key)
        # sub = rospy.Subscriber('gps/fix', NavSatFix, move_robot)
        # move_robot()()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    