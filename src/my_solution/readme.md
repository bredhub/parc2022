# TEAM : Bredhub Tech Stars

## Introduction


Agricultural robotics involves the use of automated technologies and robots to perform tasks in agricultural processes. In the context of Africa, where agriculture plays a crucial role in the economy and food security, agricultural robotics can have significant relevance. 

It can help overcome challenges such as labor shortages, increase productivity and efficiency, improve crop yield and quality, enable precise monitoring and management of crops, and empower farmers with advanced data-driven decision-making tools, ultimately contributing to sustainable agriculture and rural development in Africa.
### Task 
   The competition task entails creating code that moves a robot through a barn while dodging every obstacle, and the second task has the robot finding weeds in the barn and publishing the location to a ros topic. 

**Team Country:** Nigeria

**Team Member Names:**

* Ahamefule Michael (Team Leader)
* Ahamefule Bliss
* Ikechukwu Ayo
* Ahamufule Olive
* Ihugba Favour



## Dependencies

**Packages needed are:** (These are ONLY examples for you to follow)



* `opencv`: Open-source computer vision library.
    * `$ sudo apt-get install python-opencv`

* `tf`: transformation.
    * `$ sudo apt-get install ros-noetic-tf_transformations`
    * `$ sudo apt-get install transforms3d`

* `cv_bridge`:  convert image to cvimage.
    * `$ sudo apt-get install python-opencv`

## Task 1

    - By using the mean geolocation of two nearby pegs to create a point, I created a map using the peg locations. I made 8 points that went toward the goal.
    - I measured the distance between the robot's starting point and its intended goal for each movement. I calculate the robot's distance from its starting point and compare it to the distance from its starting point to the target location as it moves. Once the points have been updated, the current target point becomes the new initial point, and the next target point is chosen from the arrays of points if the robot's distance exceeds the target location.
    - I used the right camera, left camera, and camera sensors to take pictures of the moving robot in order to avoid obstacles. Additionally, I converted the image to hsv before threshing it. Calculate the distance to the robot after converting the detected blob to cartesian coordinates. It drifts away from the obstacle if the distance is less than 2.5.

Write the command required to run your solution. Should be in this format: <br>
` roslaunch my_solution task1_solution.launch `



## Task 2

- I used the right camera, left camera, and camera to take the pictures. I converted the image to hsv and then filtered it until I could only see weeds. I cleaned it up by eroding it. I then carry out a blob detection, obtain the centroid of the blob, and convert it to cartesian coordinates.

Write the command required to run your solution. Should be in this format: <br>
` matlab -nodesktop -nosplash -r "run('~/catkin_ws/src/my_solution/scripts/task2_solution.m')" `

## Challenges Face

### Task 1
    - the right camera sometimes reads as left camera sensor when 
    - the robot moves from position when idle
    - I had challenges with the drifting of the robot when obstruction is detected

### Task 2
    - the subscriber that reads if robot has started is slow to return response and the robot might have move far before it gets the response and it affects
