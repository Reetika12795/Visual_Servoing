
<p align="center">  
   <img src = "images/ub.png" width = 200>
</p >

# <p align="center">University of Burgundy
			
</p >

# <p align="center">Master in Computer Vision and Robotics
</p >
 
<p align="center">  
   <img src = "images/vibot.png" width = 80>
</p >

## <p align="center">Under the guidance of:</p > 
### <p align="center"> Omar TAHRI                     
</p >

## <p align="center">Team Members:</p >
### <p align="center">REETIKA GAUTAM</p>
### <p align="center">SEIKH MOHAMMED BASHARAT MONES</p> /

# <p align="center"> TABLE OF CONTENT </p>

 1. [Introduction](https://github.com/Reetika12795/Visual_Servoing#introduction)
	1. [Architecture of our Approach](https://github.com/Reetika12795/Visual_Servoing#architechture-of-our-model)
	
 1. [Camera Calibration](https://github.com/Reetika12795/Visual_Servoing#camera-calibration)

 2. [3D pose estimation by using Aruco Marker](https://github.com/Reetika12795/Visual_Servoing#detection-of-aruco-and-3d-pose-estimation)

 3. [Transforming the system to Robot Frame](https://github.com/Reetika12795/Visual_Servoing#frame-transformations)

 4. [Control Used](https://github.com/Reetika12795/Visual_Servoing#control-loopflow-chart-req)

 5. [Integration of Aruco marker with ROS ](https://github.com/Reetika12795/Visual_Servoing#integration-of-ros)
    1) [ What is ROS: ](https://github.com/Reetika12795/Visual_Servoing#1-what-is-ros)

    2) [ What is the publisher and Subscriber](https://github.com/Reetika12795/Visual_Servoing#2-what-is-the-publisher-and-subscriber)

7. [Implementation of working code](https://github.com/Reetika12795/Visual_Servoing#3-launch-the-robot-)

    *  Launch the Robot
    * Launch the Ueye Camera
    * Launch the Aruco Markers to get the pose
    * rosrun the Control loop
    * [Demo video of robo parking](https://github.com/Reetika12795/Visual_Servoing#demo-video-of-robot-parking)
    * [Demo video of robo as learder follower](https://github.com/Reetika12795/Visual_Servoing#demo-video-of-visual-servoing)
    

 7. [Obstacle avoidance using Lidar Data](https://github.com/Reetika12795/Visual_Servoing#obstacle-avoidance)

    1) [Obstacle detection and avoidance](https://github.com/Reetika12795/Visual_Servoing#1-obstacle-detection)
    2) [Demo video](https://github.com/Reetika12795/Visual_Servoing#2-the-algorithm-developed)


 8. [Conclusion and learning outcome](https://github.com/Reetika12795/Visual_Servoing#conclusion)
 
 9. [References](https://github.com/Reetika12795/Visual_Servoing#references)




# Introduction:

Visual servoing is a control technique used in robotics to control the motion of a robot using visual feedback from a camera. It involves using visual information from the robot's environment to control the motion of its end effector, or the part of the robot that interacts with its surroundings. Visual servoing allows robots to perform tasks such as tracking a moving object, following a predetermined path, or manipulating objects in their environment. This technique can be used in a variety of applications, including manufacturing, healthcare, and search and rescue operations.

For our project we will demontrate a real time visual servoing performed on TurtleBot3- Burger to follow a defined target from an UEYE-camera using the configuration (Eye-to-hand) configuration uisng ROS noetic :

* The first objective is to move the robot from an initial position to a Target position considering the target orientation (Parking).

* The second task is to make the robot follow the optimal path to target by avoiding the obstacle encountered on it's way.

# Architechture of our model

*  Launch video feed from Ueye_cam

*  Camera calibration

*  Pose initial and target robot estimation

*  control system

*  Integration of ROS

*  Obstacle avoidance
![alt text](images/timeline.png)

# Camera Calibration:

Camera Calibration is the process of estimating the intrinsic and extrinsic parameters. Intrinsic parameters refer to the internal characteristics of the camera, such as focal length, tilt, distortion, and image center. The extrinsic parameters describe the position and its orientation in the real (X, Y, Z) frame.

![alt text](images/calib.png)

**This is the first step we must take before doing anything else.**

![](https://user-images.githubusercontent.com/76461363/206848564-6d615ea7-130a-4955-9420-ed19bc3ba407.png)

![](https://user-images.githubusercontent.com/76461363/206848672-9af177ee-715b-492c-9d05-796672afff5e.png)
**The intrinsic parameters are:**

_f_: the focal length.

_Ku, Kv_: the image magnification factors.

_Cu, Cv:_ the coordinates of the projection of the optical center of the camera on the image plane.

_Suv:_ which reflects the potential non-orthogonality of rows and columns of electronic cells that make up the camera sensor. Most of the time, this parameter is neglected and therefore takes a zero value.

**The extrinsic parameters are:**

![](https://user-images.githubusercontent.com/76461363/206848708-e36be739-bb3e-47b1-bfdb-e29391cff67b.png)

**_R3x3_**: which is the rotation matrix allowing to pass from the reference frame linked to the working space to the reference frame linked to the camera.

_tx ty tz_: which are the components of the translation vector allowing to pass from the reference frame linked to the working space to the reference frame linked to the camera.

In total, there are 12 parameters to be estimated (the rotation matrix R contains 9 elements, but the relation that defines it as a rotation matrix R. RT = 1 reduces the number of independent elements to 3: the 3 polar angles).
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image014.png)

To calibrate our camera we camera_calibration ROS package as shown below.

And it generates a zip file with all the images taken during the calibration and also a .yaml file which contains the calibrated camera parameters.

To run this command in terminal:

```bash
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.12 image:=/camera/image_raw camera:=/camera
```

**Example of camera calibration data matrix:**

![alt text](images/calibration_data.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image026.png)

# Detection of aruco and 3D pose estimation

Pose estimation refers to the process of determining the orientation and position of an object in space. This can be useful for a variety of applications, such as augmented reality, robotics, and computer vision.

The objective of this step is to know the initial position of the robot and the target position to plan the trajectory.  A closed loop system based on visual servoing needs real time detection to minimize the error between the objective and the current position.

There are several methods to detect the pose using the camera for example Qrcodes, Aruco markers, or directly using image processing using some filters for detecting the depth of the image etc.

For our model we use the my_aruco package for detection of two markers, one for robot and one for target resoectively . Then, two methods were tested to extract the correct position of the aruco as well as its orientation.

![alt text](images/BothARUCOwithPose.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image028.png)


A real-world problem is a 3D problem, because of the real condition of the workspace. So, a 3D position is necessary to move the robot in an efficient way. To have this 3D position we are using my_aruco_package  to get pose  of two specific markers, in our case(25 and 701)

![alt text](images/MicrosoftTeams-image.png)

Using this information, a transformation matrix can be calculated by computing the rotation matrix from the rotation vector (using Rodriguez function) and then build the transformation matrix.

![](https://user-images.githubusercontent.com/76461363/206848984-defa72d0-ed6a-4153-bbb6-c538a01ccd5a.png)

Using two aruco markers with different ID, we can identify the current robot and the target position in real-time. This method has two advantages:

* 3D position, which allows to solve a 3D navigation.

* Detecting the target and the robot in real-time, which allows the robot to reach the target even if the target is moving (dynamic control).

# Frame transformations

In this step, two transformation matrices are available, one for the current position, and the other for the target position in the camera frame. The objective is to get the target position in the robot frame, so we have all the coordinates and the angles in this frame.

![](https://user-images.githubusercontent.com/76461363/206848991-c9a72058-010f-4575-a4a0-cb51decc598e.png)


Supposing that: The transformation matrix from the target to the camera is Tcamera_target, and the transformation matrix to the robot is Tcamera_robot, so the Transformation matrix from the target to the robot :  ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image036.png) 
![alt text](images/equation.png)

To do so, we need to combine these two matrices by multiplying the inverse of the [current transformation matrix] of the robot by the [target transformation matrix] to receive the combined transformation matrix (t) which is called:

![](https://user-images.githubusercontent.com/76461363/206848993-bd079be2-cb68-4bbf-a315-ebc8a080d4b9.png)




In python we did that using the following code:

```python
np.linalg.inv(Robot_matrix_transformation).dot(Target_matrix_transformation)
```

# Control Loop

To move the TurtleBot, we need to send the order in a form of value of speed. 3 linear speeds and 3 rotational speeds in xyz axis. Corresponding to the number of degrees of freedom that we have on our robot (1 on x axis, 1 on y axis and 1 for the rotation around the z axis), two linear speeds (in respect of x and y) and one rotational speed (z) are given to the robot to move to a position. Consequently, we need to calculate these speeds first and then send it to the robot. The concept is to reduce the difference between the initial position and the target position. The difference includes the distance between them and the gap between their orientations.

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image041.jpg)The distance between the two positions (ρ), the angle between the orientation of the robot and the target (θ), the angle between the orientation of the robot and the direction of the target (α)

![alt text](images/control.png)

To reduce the distance, a forward movement toward the target is required. So, a forward speed and a direction must be calculated. Two parameters are responsible (α and ρ)

The third parameter (beta) should be reduced to park the robot in the same orientation as the target.

The speeds are calculated then sent every (1/50) seconds to the robot using ROS. To have a smooth movement, max speed should be determined which allows also to take in consideration the physical constraint of the vehicle.

## Simulation to test the Control 

Simulate the control system using python code, by simulate a robot position which updates its position every time the speed is calculated. The next position is calculated by these equations:
 put equation from slides.

Here we see that our intitial position is defined as [0,0,-90] and we see clearly that our robot to reach the target takes a curved path to justify our control and sucessfully reaches the target of [2,2]
![alt text](images/simulation.png)


# Integration of ROS

## 1.    What is ROS:

ROS (Robot Operating System) is an open-source software development kit for robotics applications. ROS provides developers across all industries with a standard software platform that enables them to move from research and prototyping to deployment and production. ROS has a concept **Don't reinvent the wheel. Create new things and make them faster and better by building on ROS!**

## 2.    What is the publisher and Subscriber:

Message passing in ROS happens with the Publisher Subscriber Interface provided by ROS library functions.

A ROS Node can be a Publisher or a Subscriber. A Publisher is the one puts the messages of some standard Message Type to a particular Topic. The Subscriber on the other hand subscribes to the Topic so that it receives the messages whenever any message is published to the Topic.

Note that a publisher can publish to one or more Topic and a Subscriber can subscribe to one or more Topic. Also, publishers and subscribers are not aware of each other’s existence. The idea is to decouple the production of information from its consumption and all the IP addresses of various nodes are tracked by the ROS Master.

![](https://thanhnguyensite.files.wordpress.com/2020/11/ac627-0wpj6rtkf1igna0m2.png)

<br>

## 3. Launch the Robot :

The robot is already delivered with all the necessary ROS packages, otherwise we can easily get them by flowing a good tutorial made by ‘ROBOTIS’ [https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

To launch the robot in the terminal, write these commands:
```
roscore
```
We need to remotely connect to the bot with local server and run the following commands
```
ssh [user]@[ip_address] #of the bot
```
After logging in to bot run: 
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

The topics to move the robot and controls will be up until this point.

<br>

## 4. Launch the Camera :
<br>

To publish our code in the robot, we must get the images of the camera first. In order to do that, there is already ROS packages of the U-eye camera. We can download and install them from: [https://github.com/anqixu/ueye_cam](https://github.com/anqixu/ueye_cam)

After successful installation we run the following command to check the view of camera:
```
roslaunch ueye_cam rgb8.launch
```
<br>

## 5. Detect ArUco Marker :
Aruco marker detection is done with ROS ArUco package which was developed by [Pal-Robotics](https://github.com/pal-robotics/aruco_ros).

The package helps to detect the aruco marker in the camera frame.
For different aruco markers, we have created different launch files for avoiding any conflicts between data.

After cloning the repository proviously mentioned and installation, go inside the launch folder, create a new launch file for different aruco marker and put the aruco marker ID and size of the marker or provide the same while running the launch file as arguments.

![image](https://user-images.githubusercontent.com/116564367/207371246-7de48631-9e39-44d5-9731-af32b2cf6d28.png)

To launch the marker 25 for the robot, launch in RemotePC :
```
roslaunch my_aruco_tracker aruco_marker_finder.launch
```

To launch the marker 701 for the target, launch :
```
roslaunch my_aruco_tracker aruco_marker_finder_2.launch
```
It will show the detected marker with it's ID successfully.

![image](https://user-images.githubusercontent.com/116564367/207372561-49861012-3d11-4e39-91e1-e8fbe74555b9.png)

<br>

## 6. Code to detect the pose from ArUco maarkers :

'aruco_marker_finder.launch' launch file will detect and provide the data of the pose of the marker. This is described in an earlier section.

To get the pose we have used 2D coordinates as our bot is moving in 2D.
Run the following snippet to get the pose from the marker :

```
rosrun my_aruco_tracker get_aruco_coord.py
```
![image](https://user-images.githubusercontent.com/116564367/207375083-1597c225-a4a2-4e84-b7d4-e94ac84c0336.png)

<br>

## 7. Generate transformation matrix

Transformation matrix provides the transformation between two poses.
To get the transformation matrix run:

```
rosrun my_aruco_tracker transformation_matrix.py
```

## 8. Move the robot :

After we get the transformation matrix, it is the time to move the robot with calculated data.
The bot will move from initial position which is the position of robot itself and move to a destination which can be mobile or a fixed position.

This step is to publish the speed calculated by publishing it on the robot’s “**cmd_vel**” topic. A servoing closed loop that gets the real time image, calculates the velocity and sends it to the robot. The robot receives the instruction and moves to the target. The robot stops when the distance between the current and the target is about 20cm.

## Move the robot

```python
rosrun my_aruco_tracker path_plan_1.py
```
## **NOTE : By runnig path_plan.py, we are running all the previous files to get aruco coordinates and calculation of transformation matrix. So there is no need to run the previous two scripts, 'get_aruco_coord.py' and 'transformation_matrix.py'**

<br>

# Demo video of Robot Parking


https://user-images.githubusercontent.com/116564367/207425999-63dfdd6a-4dc8-4056-9795-222de2fbb4dd.mp4


# Demo video of Visual Servoing

https://user-images.githubusercontent.com/116564367/207425777-81ebaaec-93d1-4d60-be42-060b9ed519db.mp4


# Obstacle avoidance

Detect and avoid obstacles are important features for a mobile robot to navigate in an unknown environment. First step is detecting the obstacles where simple object detection or deep learning can be used. Second step is the path planning to reach the target.

We have used lidar to avoid obstacle in our algorithm.

## 1. Obstacle detection

Detection of an obstacle is an important part of any self-driving robot. Lidar is a very reliable device to calculate distance from nearby field within its detectable range. The lidar in turrtlebot3 has a range of 120 ~ 3,500mm.


https://user-images.githubusercontent.com/116564367/207426348-3f00c999-504b-49d9-93dc-413128de6fb1.mp4


## 2. The algorithm developed

We have read the lidar data on specific angles which are 0 degree, 15 degrees and 345 degrees.

0 degree is the front of the turtlebot3 and it measures in an anticlockwise manner.
The data we get from turtlebot3 lidar is continious but not proper reliable as the data is ambiguous. While scanning the surrounding, the lidar sends lots of zeros and it makes the detection difficult in real time.

Different turtlebots have been tried to overcome this problem but it seems that they are made out of same mother.

A threshold of 300mm is kept for avoidance of obstacles which is, if any object is within the range of 300mm of the robot at 0, 15, 345 degrees, the robot will make it's provided direction other than the calculated direction from the transformation matrix.



https://user-images.githubusercontent.com/116564367/207426650-6daa5a5e-d7d2-48e0-9f54-29c8a6dd0bbf.mp4

# Conclusion

Controlling a robot in an unknown environment add more challenge to the control system and to the detection. Detecting the robot will not be as easy as while using the aruco marker. Moving the robot in a rugged terrain needs a robust control system that takes in consideration the tough surface and the sliding of the wheels.

The project allowed us to take on hand several important robotic skills, image processing, visual servoing, path planning, interpretation of the result, frame transformation, and soft skills as well.

# References

[ROS TurtleBot3 Basics](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)  
[ROS Wiki](http://wiki.ros.org/noetic)  
[ROS Basics course](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/ros-perception-in-5-days/)  
[ROS perception course](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/ros-perception-in-5-days/)
