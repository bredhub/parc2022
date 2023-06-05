#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition task must be running for this code to work.

import math

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from parc_robot.gps2cartesian import gps_to_cartesian
from sensor_msgs.msg import (CameraInfo, Image, LaserScan, NavSatFix,
                             PointCloud2)
from tf.transformations import euler_from_quaternion

# Register the coordinates of the pegs
peg_01 = rospy.get_param('peg_01')
peg_02 = rospy.get_param('peg_02')
peg_03 = rospy.get_param('peg_03')
peg_04 = rospy.get_param('peg_04')
peg_05 = rospy.get_param('peg_05')
peg_06 = rospy.get_param('peg_06')
peg_07 = rospy.get_param('peg_07')
peg_08 = rospy.get_param('peg_08')

peg_01_lat, peg_01_lon = peg_01['latitude'], peg_01['longitude']
peg_02_lat, peg_02_lon = peg_02['latitude'], peg_02['longitude']
peg_03_lat, peg_03_lon = peg_03['latitude'], peg_03['longitude']
peg_04_lat, peg_04_lon = peg_04['latitude'], peg_04['longitude']
peg_05_lat, peg_05_lon = peg_05['latitude'], peg_05['longitude']
peg_06_lat, peg_06_lon = peg_06['latitude'], peg_06['longitude']
peg_07_lat, peg_07_lon = peg_07['latitude'], peg_07['longitude']
peg_08_lat, peg_08_lon = peg_08['latitude'], peg_08['longitude']



def mean_gps(x1,y1, x2, y2):
        # Convert degrees to radians
        lat1 = math.radians(x1)
        lon1 = math.radians(y1)
        lat2 = math.radians(x2)
        lon2 = math.radians(y2)

        # Find the average of the latitudes and the longitudes
        avg_lat = (lat1 + lat2) / 2
        avg_lon = (lon1 + lon2) / 2

        # Convert back from radians to degrees
        midpoint_latitude = math.degrees(avg_lat)
        midpoint_longitude = math.degrees(avg_lon)

        return [midpoint_latitude, midpoint_longitude]
    
def mean_average(self, pointA, pointB):
    x1, y1 = pointA
    x2, y2 = pointB
    mid_x = (x1+x2)/2
    mid_y = (y1 + y2)/ 2
    return [mid_x, mid_y]
  
point1_gps = mean_gps(peg_01_lat, peg_01_lon, peg_04_lat, peg_04_lon)
point2_gps = mean_gps(peg_02_lat, peg_02_lon, peg_03_lat, peg_03_lon)
point3_gps = mean_gps(peg_03_lat, peg_03_lon,peg_06_lat, peg_06_lon)
point4_gps = mean_gps(peg_04_lat, peg_04_lon,peg_05_lat, peg_05_lon)
point5_gps = mean_gps(peg_05_lat, peg_05_lon,peg_08_lat, peg_08_lon)
point6_gps = mean_gps(peg_07_lat, peg_07_lon,peg_06_lat, peg_06_lon)
#gps to cartesian
point1 = mean_average(gps_to_cartesian(peg_01_lat, peg_01_lon), gps_to_cartesian(peg_04_lat, peg_04_lon))
point2 = mean_average(gps_to_cartesian(peg_02_lat, peg_02_lon), gps_to_cartesian(peg_03_lat, peg_03_lon))
point3 = mean_average(gps_to_cartesian(peg_03_lat, peg_03_lon), gps_to_cartesian(peg_06_lat, peg_06_lon))
point4 = mean_average(gps_to_cartesian(peg_04_lat, peg_04_lon), gps_to_cartesian(peg_05_lat, peg_05_lon))
point5 = mean_average(gps_to_cartesian(peg_05_lat, peg_05_lon), gps_to_cartesian(peg_08_lat, peg_08_lon))
point6 = mean_average(gps_to_cartesian(peg_07_lat, peg_07_lon), gps_to_cartesian(peg_06_lat, peg_06_lon))
goal_location = rospy.get_param('goal_location')
all_points = [point2, point3, point4, point4, point5, goal_location]
current_index_target = 0
current_goal = all_points[current_index_target]
main_goal_met = False
desired_angular_vel = 0.0

def sensor_lidar():
    scan_data = rospy.wait_for_message('/scan', LaserScan)
    return scan_data

def left_camera():
    scan_data = rospy.wait_for_message('/left_camera/image_raw', Image)
    return scan_data

def right_camera():
    scan_data = rospy.wait_for_message('/right_camera/image_raw', Image)
    return scan_data

def right_camera_info():
    scan_data = rospy.wait_for_message('/right_camera/camera_info', CameraInfo)
    return scan_data

def left_camera_info():
    scan_data = rospy.wait_for_message('/left_camera/camera_info', CameraInfo)
    return scan_data

def point_cloud():
    scan_data = rospy.wait_for_message("/zed2/point_cloud/cloud_registered", PointCloud2)
    return scan_data
        
def odom():
    scan_data = rospy.wait_for_message('odom', Odometry)
    return scan_data

def check_robot_orientation(data):
    orientation = data.pose.pose.orientation
        
    # Convert the orientation to Euler angles
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    # Update the robot's current orientation (theta)
    return yaw

def gps():
    gps_msg = rospy.wait_for_message('/gps/fix', NavSatFix)
    current_lat = gps_msg.latitude
    current_lon = gps_msg.longitude
    x, y = gps_to_cartesian(current_lat, current_lon)
    return [x, y]

def think(scan_data, robot_position):
    global desired_angular_vel
    fwd_range = scan_data.ranges[190:210]
    distance_to_wall = sum(fwd_range)/len(fwd_range)
    approach_threshold = 0.1
    
    
    if distance_to_wall > approach_threshold:
        #get where robot is pointing to
        orientation = robot_position.pose.pose.orientation
        
        # Convert the orientation to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if yaw <=0:
            desired_angular_vel = 0.05
        elif yaw >= 0:
            desired_angular_vel = -0.05
        
        move_flag = True
    else:
        move_flag = False

    print(f'Distance to wall: {distance_to_wall}')

    return move_flag  

def calculate_desired_heading(current_lat, current_lon):
    global current_location
    # Calculate the angle between the current position and the goal position
    goal_lat = current_location[0]
    goal_lon = current_location[1]
    delta_lat = goal_lat - current_lat
    delta_lon = goal_lon - current_lon

    # Convert delta_lat and delta_lon to radians
    delta_lat_rad = math.radians(delta_lat)
    delta_lon_rad = math.radians(delta_lon)

    # Calculate the desired heading angle
    desired_heading = math.atan2(delta_lon_rad, delta_lat_rad)

    return desired_heading
    
def current_goal_is_met(robot_position):
    global current_location, current_index_target, desired_angular_vel
    if((robot_position[0] >= current_location[0]) and (robot_position[1] >= current_location[1])):
        if current_index_target == 6:
            main_goal_met = True
            return True
        else:
            current_index_target = current_index_target + 1
            current_location = all_points[current_index_target]
            desired_heading = calculate_desired_heading(robot_position[0], robot_position[1])
            desired_angular_vel = desired_heading
            return True
    else:
        return False
        
    
    
def act(robot_vel_publisher, move_flag, robot_position):
    global desired_angular_vel
    
    robot_vel = Twist()
    fwd_vel = 0.3
    
    if move_flag[0]:
        if current_goal_is_met(robot_position):
            if main_goal_met:
                robot_vel.linear.x = 0.0
                robot_vel.angular.z = desired_angular_vel
                msg = "Robot stopped because main goal is reached"
                robot_vel_publisher.publish(robot_vel)
            else:
                robot_vel.linear.x = 0.0
                robot_vel.angular.z = 0.0
                robot_vel_publisher.publish(robot_vel)
                rospy.sleep(1)
                
                #turns to next goal
                robot_vel.angular.z = desired_angular_vel
                robot_vel_publisher.publish(robot_vel)
                rospy.sleep(1)
                
                #starts moving
                robot_vel.linear.x = fwd_vel
                robot_vel.angular.z = 0.0
                robot_vel_publisher.publish(robot_vel)
                
                msg = "Robot stopped because goal reached"
        else:         
            #continue movement
            robot_vel.linear.x = fwd_vel
            robot_vel.angular.z = 0.0
            msg = "Robot Moving! \n"
            robot_vel_publisher.publish(robot_vel)
 
    else:
        #stops
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = 0.0
        robot_vel_publisher.publish(robot_vel)
        rospy.sleep(1)
            
        #turns away from obstacle
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = desired_angular_vel
        robot_vel_publisher.publish(robot_vel)
        rospy.sleep(1)
            
            
        #keeps turning till it is away from obstacle
        scan_lidar = sensor_lidar()
        move_flag_again = think(scan_lidar)
            
        while not move_flag_again:
            #turns away from obstacle
            robot_vel.linear.x = 0.0
            robot_vel.angular.z = desired_angular_vel
            robot_vel_publisher.publish(robot_vel)
            rospy.sleep(1)
            
        robot_vel.linear.x = fwd_vel
        robot_vel.angular.z = 0.0
        msg = "Robot turned! because of obstacle \n"
        robot_vel_publisher.publish(robot_vel)

    return msg
    

def main():
    rospy.init_node('collision_avoidance_node')
    
    hz = 10
    rate = rospy.Rate(hz)

    robot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    
    while not rospy.is_shutdown():
        
        #sense
        scan_lidar = sensor_lidar()
        left_camera_scan = left_camera()
        right_camera_scan = right_camera()
        left_info = left_camera_info()
        right_info = right_camera_info()
        
        #get position 
        robot_position = gps()
        
        
        #where the robot is facing
        position_robot = odom()
        
        #think
        move_flag = think(scan_lidar, position_robot)
        
        #act
        message = act(robot_vel_publisher, move_flag, robot_position)
    
        
        
    

       
    
if __name__ == '__main__':
    main()