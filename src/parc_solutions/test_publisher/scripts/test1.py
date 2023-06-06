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
from sensor_msgs.msg import (CameraInfo, Image, Imu, LaserScan, NavSatFix,
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
    
def mean_average(pointA, pointB):
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
goal_location_lat = rospy.get_param('goal_latitude')
goal_location_log = rospy.get_param('goal_longitude')
goal_location = gps_to_cartesian(goal_location_lat, goal_location_log)
all_points = [point1, point2, point3, point4, point4, point5, goal_location]
prev_index_target = 0
current_index_target = 1
current_location = all_points[current_index_target]
prev_location = all_points[prev_index_target]
main_goal_met = False
desired_angular_vel = 0.0
current_orientation = None
keypoints = []
last_image = None


def sensor_lidar():
    scan_data = rospy.wait_for_message('/scan', LaserScan)
    return scan_data

def sensor_zed():
    scan_data = rospy.wait_for_message('/zed2/imu/data', Imu)
    return scan_data

def left_camera():
    scan_data = rospy.wait_for_message('/left_camera/image_raw', Image)
    return scan_data

def calculate_distance_to_blob(robot_position, blob_position):
    robot_x, robot_y = robot_position
    blob_x, blob_y = blob_position

    distance = math.sqrt((blob_x - robot_x)**2 + (blob_y - robot_y)**2)
    return distance

def estimate_distance(cv_image, focal, robot_position):
    global keypoints
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Apply a threshold to convert the grayscale image to binary
    COLOR_MIN = (39, 106, 124)
    COLOR_MAX = (77, 237, 189)
    thresh_img = cv2.inRange(gray_image, COLOR_MIN, COLOR_MAX)
    
    params = cv2.SimpleBlobDetector_Params()
    # Set parameters for blob detection (change as needed)
    params.minThreshold = 10
    params.maxThreshold = 200
    params.filterByArea = True
    params.minArea = 100
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)
    
    # Detect blobs
    keypoints = detector.detect(thresh_img)
    
    image_with_blobs = cv2.drawKeypoints(thresh_img, keypoints, np.array([]), (0, 0, 255),
                                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Display the image with blobs (for visualization purposes)
    cv2.imshow("Obstacle Detection", image_with_blobs)
    cv2.waitKey(1)
    print(keypoints)
    obstacle_detected = len(keypoints) > 0
    keypoints = keypoints
    robot_x = robot_position[0]
    
    robot_y = robot_position[1]
    obstacle_distances = []
    for keypoint in keypoints:
        blob_x, blob_y = keypoint.pt
        distance_to_blob = math.sqrt((blob_x - robot_x)**2 + (blob_y - robot_y)**2)
        obstacle_distances.append(distance_to_blob)
    print(obstacle_distances)
    print("obstacel")
    if obstacle_detected:
        min_distance = min(obstacle_distances)
        # Check if the minimum distance is below the obstacle distance threshold
        if min_distance < 638:
            # Perform obstacle avoidance actions
            # Example: Stop the robot, change direction, etc.
            print("Obstacle detected. Taking avoidance action.")
            return True
    
    return False


def right_camera():
    scan_data = rospy.wait_for_message('/right_camera/image_raw', Image)
    return scan_data

def camera():
    scan_data = rospy.wait_for_message('/camera/image_raw', Image)
    return scan_data

def analyse_image(scan_data, camera_info, robot_position):
    if camera_info is None:
        rospy.logwarn('Camera info not available yet.')
        return False
    
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(scan_data, desired_encoding='bgr8')
        turn = False
        
        # Perform image processing and distance estimation
        obstacle_detected = estimate_distance(cv_image, camera_info, robot_position)
        return obstacle_detected
    except Exception as e:
        rospy.logerr(f"Error processing image: {str(e)}")
        return False
    
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

def think(scan_data, robot_position=None):
    global desired_angular_vel
    approach_threshold = 0.161
    ranges = scan_data.ranges
    front_distance = min(ranges[int(len(ranges)/2-10):int(len(ranges)/2+10)])
    
    print("min rane"+ str(front_distance))
    if front_distance > approach_threshold:
        # Distance to obstacle is infinity, move towards the goal
        desired_angular_vel = 0.0
        move_flag = True
    else:
        scanner = sensor_lidar()
        ranges = scanner.ranges
        left_distance = min(ranges[270:360])  # Measure distance in the left region
        right_distance = min(ranges[0:90])  # M
        
        if left_distance > right_distance:
            desired_angular_vel = -0.03
        else:
            desired_angular_vel = 0.03
        
        move_flag = False
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

def distance_between_two_point(pointA, pointB):
    # Calculate the distance between two states - we are only interested in the x and y coordinates
    return ((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2) ** 0.5


def current_goal_is_met(robot_position):
    global current_location,prev_index_target, prev_location, current_index_target, desired_angular_vel
    # print(robot_position)
    # print(current_location)
    # print("--------------------")
    initial_point_final = distance_between_two_point(prev_location, current_location)
    robot_goal = distance_between_two_point(robot_position, current_location)
    robot_initial = distance_between_two_point(robot_position, prev_location)
    # print(initial_point_final)
    # print(robot_goal)
    # print("-----------------")
    if((robot_initial >= initial_point_final)):
        if current_index_target == 6:
            main_goal_met = True
            return True
        else:
            current_index_target = current_index_target + 1
            prev_index_target = prev_index_target + 1
            prev_location = all_points[prev_index_target]
            current_location = all_points[current_index_target]
            # desired_heading = calculate_desired_heading(robot_position[0], robot_position[1])
            # desired_angular_vel = desired_heading
            return True
    else:
        return False
        
    
    
def act(robot_vel_publisher, move_flag, robot_position, right_call, left_call):
    global desired_angular_vel
    
    robot_vel = Twist()
    fwd_vel = 0.2
    
    if right_call:
        #stop
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = 0.0
        msg = "Robot stopped turn left"
        robot_vel_publisher.publish(robot_vel)
        rospy.sleep(1)
        
        #chnae direction
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = -0.07
        robot_vel_publisher.publish(robot_vel)
        rospy.sleep(1)
        
        #keep moving
        robot_vel.linear.x = 0.2
        robot_vel.angular.z = 0.0
        robot_vel_publisher.publish(robot_vel)
        
        
    elif left_call:
        #stop
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = 0.0
        msg = "Robot stopped because turn right"
        robot_vel_publisher.publish(robot_vel)
        rospy.sleep(1)
        
        #chnae direction
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = 0.07
        
        robot_vel_publisher.publish(robot_vel)
        rospy.sleep(1)
        
        #keep moving
        robot_vel.linear.x = 0.2
        robot_vel.angular.z = 0.0
        robot_vel_publisher.publish(robot_vel)
        
    elif move_flag:
        if current_goal_is_met(robot_position):
            if main_goal_met:
                robot_vel.linear.x = 0.0
                robot_vel.angular.z = 0.0
                msg = "Robot stopped because main goal is reached"
                robot_vel_publisher.publish(robot_vel)
               
            else:
                robot_vel.linear.x = 0.2
                robot_vel.angular.z = 0.0
                robot_vel_publisher.publish(robot_vel)
                rospy.sleep(1)
                
                # #turns to next goal
                # robot_vel.angular.z = desired_angular_vel
                # robot_vel_publisher.publish(robot_vel)
                # rospy.sleep(1)
                
                
                # #stop
                # robot_vel.linear.x = 0.0
                # robot_vel.angular.z = 0.0
                # robot_vel_publisher.publish(robot_vel)
                # rospy.sleep(1)
                
                #starts moving
                # robot_vel.linear.x = fwd_vel
                # robot_vel.angular.z = 0.0
                # robot_vel_publisher.publish(robot_vel)
                
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
        print("stops to turn")
        rospy.sleep(1)
            
        #turns away from obstacle
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = desired_angular_vel
        robot_vel_publisher.publish(robot_vel)
        print("turns away from obstacle")
        rospy.sleep(1)
        
        
        #stops
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = 0.0
        robot_vel_publisher.publish(robot_vel)
        print("stops")
        rospy.sleep(1)
            
        # #keeps turning till it is away from obstacle
        # scan_lidar = sensor_lidar()
        # move_flag_again = think(scan_lidar)
            
        # while not move_flag_again:
        #     print("checking again")
        #     #turns away from obstacle
        #     robot_vel.linear.x = 0.0
        #     robot_vel.angular.z = desired_angular_vel
        #     robot_vel_publisher.publish(robot_vel)
        #     rospy.sleep(1)
        
        
        
        
        #continue moving
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
        camera_scan = camera()
        left_camera_scan = left_camera()
        right_camera_scan = right_camera()
        left_info = left_camera_info()
        right_info = right_camera_info()
        
        #get position 
        robot_position = gps()
        
        
        #where the robot is facing
        position_robot = odom()
        
        #front camera
        front_call = analyse_image(camera_scan, right_info, robot_position)
        #sense item by right
        right_call = analyse_image(right_camera_scan, right_info, robot_position )
        right_call = analyse_image(right_camera_scan, right_info, robot_position)
        
        # #sense item by left
        left_call = analyse_image(left_camera_scan, left_info, robot_position)
        
        #think
        # move_flag = think(scan_lidar, position_robot)
        
        move_flag = False
        #act
        message = act(robot_vel_publisher, move_flag, robot_position, right_call, left_call)
    
        
        print(f'At time []: {message}!')

        rate.sleep()
    

       
    
if __name__ == '__main__':
    main()