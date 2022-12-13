## MSFT PROJECT MSCV


<p align="center">  
   <img src = "images/ub.png" width = 200>
</p >

#  <p align="center">University of Burgundy</p >
#  <p align="center">**Master of Computer vision and Robotics**</p >
<p align="center">  
   <img src = "images/vibot.png" width = 80>
</p >



# <p align="center">**Under the guidance of**<br/><br/>Omar TAHRI


## <p align="center">Team Members:<br/><br/>SEIKH Mohammed Basharat Mones<br/>Reetika GAUTAM








# TABLES OF CONTENTS:



 1. [Introduction](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#introduction)

 2. [Camera Calibration](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#camera-calibration)

 3. [3D pose estimation by using Aruco Marker](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#detection-and-pose-estimation)

 4. [Transforming the system to Robot Frame](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#frame-transformation)

 5. [Control Used](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#control-system)

 6. [Integration of Aruco marker with ROS ](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#integration-of-ros)
    1) [ What is ROS: ](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-what-is-ros)

    2) [ What is the publisher and Subscriber](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-what-is-the-publisher-and-subscriber)

7. [Implementation of working code]

    1) [Launch the Robot](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#3-launch-the-robot)
    2) [Launch the Ueye Camera](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#4-launch-the-camera)
    3) [Launch the Aruco Markers to get the pose]
    4) [rosrun the Control loop]
    5) [ Demo video of robo parking]
    6) [Demo video of robo visual servoing ghoomar ghoomar]
    

 8. [Obstacle avoidance using Lidar Data](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#obstacle-avoidance)

    1) [Obstacle detection and avoidance](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-obstacle-detection)
    2) [Demo video]
    3) [The ambiguity in the LIdar Data](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-the-algorithm-developed)
    4) [Final Demo of Robot avoiding the obstacle](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#3-similar-rrt-algorithm)

 9. [Conclusion and learning outcome](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#conclusion)
 
 10. [References]




# Introduction:

During the Multi-Sensor Fusion and Tracking module we had a project to realize, the goal of this project is to control a 2-wheel mobile robot "Turtlubot3" under ROS melodic from an UEYE-camera using the configuration (Eye-to-hand):

·        The first objective is to move the robot from an initial position to a Target position considering the target orientation (Parking).

·        The second objective is to do the same thing as task 1 but with obstacles, the task of the robot is to avoid these obstacles (in our case we used buckets of a red color) and reach the target.

To solve our problem, we have divided the spots as objectives as can be seen in figure 1:

·        Camera calibration

·        Pose initial and target robot estimation

·        control system

·        Integration of ROS

·        Obstacle avoidance

![](https://user-images.githubusercontent.com/76461363/206848388-45c6d0bb-b566-4624-8acb-7dbebc14adbc.png)

![](https://user-images.githubusercontent.com/76461363/206848522-87638424-25fb-4ea0-b68d-5973452a468b.png)

# Camera Calibration:

Camera Calibration is the process of estimating the intrinsic and extrinsic parameters. Intrinsic parameters refer to the internal characteristics of the camera, such as focal length, tilt, distortion, and image center. The extrinsic parameters describe the position and its orientation in the real (X, Y, Z) frame.

**This is the first step we must take before doing anything else.**

![](https://user-images.githubusercontent.com/76461363/206848564-6d615ea7-130a-4955-9420-ed19bc3ba407.png | width=100)

![](https://user-images.githubusercontent.com/76461363/206848672-9af177ee-715b-492c-9d05-796672afff5e.png | width=100)
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

**Example of camera calibration (Distortion Elimination):**
![](https://user-images.githubusercontent.com/76461363/206848758-15ceecfe-e786-4db3-91c4-67c49b81de3f.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image014.png)

**Code Source:**

we calibrate the camera with two methods, the first one we build a code using some function of OpenCV in python:

We have used a chessboard 7x5 for the calibration.

```python
#To Find the chess board corners
ret, corners = cv.findChessboardCorners(gray, (7,5), None)

```

we define Criteria (the maximum number of iterations and/or the desired accuracy), we use it for Detecting corner’s location in subpixels, in our case we made:

```python
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
```

To generate the camera calibration there is a function of OpenCV **CalibrateCamera**, where we pass on it the object point (7*5x3), and the corners of the images contain the chessboard.!
```python
_, mtx, dist,_,_ = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
```

The function generates for us the camera matrix and distortion parameters and the translations and rotations on each image.

**The results:**

we find the following results for our camera; we save it in a yaml file using the command:

![](https://user-images.githubusercontent.com/76461363/206848928-91125865-05bf-4027-9433-5eae101e58d4.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image019.png)

We save these parameters in a yaml file using this command:

```python
yaml.dump(cal_data,f)
```

**2nd Method:**

There are several methods to calibrate the camera more accurately. Such us “the Single Camera Calibrator App” of MATLAB or also the packages of “camera_calibration” in Ros. In our case we used the second one because it’s easy to use and it takes the images of the chessboard autonomously.

And it generates a zip file with all the images taken during the calibration and also a .yaml file which contains the calibrated camera parameters.

To run this command in terminal:

```terminal
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.12 image:=/camera/image_raw camera:=/camera
```

**Result:**

we find the following results for our camera. ![](https://user-images.githubusercontent.com/76461363/206848934-9e2b89cc-abe8-433c-8ff3-188b27e97740.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image026.png)

# Detection and pose estimation

The objective of this step is to know the initial position of the robot and the target position to plan the trajectory.  A closed loop system based on visual servoing needs real time detection to minimize the error between the objective and the current position.

There are several methods to detect the pose using the camera for example Qrcodes, Aruco markers, or directly using image processing using some filters for detecting the depth of the image etc...

In our case we used an aruco marker attached to the top of the robot, we used the aruco library of OpenCV to simplify the detection. Then, two methods were tested to extract the correct position of the aruco as well as its orientation.

![](https://user-images.githubusercontent.com/76461363/206848967-f82fab51-d3bc-45bb-a5c6-13a9c39b9809.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image028.png)

## 1.    Approximation 2D without pose estimation

Here, we considered that our problem is always a 2D problem. The robot will move in x and y direction without considering the depth. In this approximation, the calibration matrix is not required. We directly detect the aruco markers in the camera using a simple python function given by OpenCV “aruco.detectMarkers” returns the position in the image (in pixel) of the 4 corners of the marker in order. We create a function called get_pose(img) in utils.py, this function includes the detectMarkers function, and it return the position of the center (x,y) of the marker and its orientation by calculating the orientation angle of the line between two points of the 4 corners of the marker  : arctan(Δy/Δx). After having the position and the orientation of the marker (current position of the robot and of the target), we calculate the transformation matrix for both positions. following this formula:

![](https://user-images.githubusercontent.com/76461363/206848977-c2432719-3bb4-43b4-9b8b-29c865d16f93.png)
![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image029.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image031.png)

## 2.    3D pose estimation

A real-world problem is a 3D problem, because of the real condition of the workspace. So, a 3D position is necessary to move the robot in an efficient way. To have this 3D position we can use another OpenCV function “estimatePoseSingleMarkers”

This function takes in input:  the corners of the marker, the marker size (in cm), the camera matrix and the distortion coefficients of the camera.  And it returns rotation vector and a translation vector (x,y,z) of a marker.

![](https://user-images.githubusercontent.com/76461363/206848984-defa72d0-ed6a-4153-bbb6-c538a01ccd5a.png)
Using this information, a transformation matrix can be calculated by computing the rotation matrix from the rotation vector (using Rodriguez function) and then build the transformation matrix.

Using two aruco markers with different ID, we can identify the current robot and the target position in real-time. This method has two advantages:

·        3D position, which allows to solve a 3D navigation.

·        Detecting the target and the robot in real-time, which allows the robot to reach the target even if the target is moving (dynamic control).

# Frame transformation

In this step, two transformation matrices are available, one for the current position, and the other for the target position in the camera frame. The objective is to get the target position in the robot frame, so we have all the coordinates and the angles in this frame.

![](https://user-images.githubusercontent.com/76461363/206848991-c9a72058-010f-4575-a4a0-cb51decc598e.png)

Supposing that: The transformation matrix from the target to the camera is Tcamera_target, and the transformation matrix to the robot is Tcamera_robot, so the Transformation matrix from the target to the robot :  ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image036.png)  

To do so, we need to combine these two matrices by multiplying the inverse of the current transformation matrix of the robot by the target transformation matrix to receive the combined transformation matrix (t) which is called:

![](https://user-images.githubusercontent.com/76461363/206848993-bd079be2-cb68-4bbf-a315-ebc8a080d4b9.png)

In python we did that using the following code:

```python
np.linalg.inv(Robot_matrix_transformation).dot(Target_matrix_transformation)
```

# Control system

To move the TurtleBot, we need to send the order in a form of value of speed. 3 linear speeds and 3 rotational speeds in xyz axis. Corresponding to the number of degrees of freedom that we have on our robot (1 on x axis, 1 on y axis and 1 for the rotation around the z axis), two linear speeds (in respect of x and y) and one rotational speed (z) are given to the robot to move to a position. Consequently, we need to calculate these speeds first and then send it to the robot. The concept is to reduce the difference between the initial position and the target position. The difference includes the distance between them and the gap between their orientations.

![](https://user-images.githubusercontent.com/76461363/206848999-83dfadeb-ef98-486f-b88c-a8cd0509c822.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image041.jpg)The distance between the two positions (ρ), the angle between the orientation of the robot and the target (θ), the angle between the orientation of the robot and the direction of the target (α)

![Picture1](https://user-images.githubusercontent.com/76461363/206851398-bc761c88-77ba-436c-a964-ffa45d197769.png)
To reduce the distance, a forward movement toward the target is required. So, a forward speed and a direction must be calculated. Two parameters are responsible (α and ρ)

The forward speed is: 

The third parameter (beta) should be reduced to park the robot in the same orientation as the target.

The speeds are calculated then sent every (1/50) seconds to the robot using ROS. To have a smooth movement, max speed should be determined which allows also to take in consideration the physical constraint of the vehicle.

## Simulation to test the control system

Simulate the control system using python code, by simulate a robot position which updates its position every time the speed is calculated. The next position is calculated by these equations:

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image061.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image063.png)

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image065.png)

The operation is repeated in a loop until the distance becomes less than 0.01 unit.

With four different initial positions, [0,0], [20,0], [0,20], [20,20], and the initial orientation of 0° degree. The target position is [10,10] and the orientation of 180° degrees.

Gain values: ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image067.png) we obtain the following trajectory graph. Noting that the gain gives importance to the parameter; giving 3.5 as a value for beta gives the parking task more importance than the 2.5 value (in absolute value).

![](https://user-images.githubusercontent.com/76461363/206849008-0a20f112-7328-4519-93b3-4ab900700c30.png)


The trajectory obtained is not optimist for all the positions, which can be explained by the fact that in this simulation the robot is not taking on consideration it’s orientation correctly as the real world.

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

To launch the marker, run :
```
roslaunch my_aruco_tracker aruco_marker_finder.launch
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

To move the robot, run:

```
rosrun my_aruco_tracker path_plan.py
```
## **NOTE : By runnig path_plan.py, we are running all the previous files to get aruco coordinates and calculation of transformation matrix. So there is no need to run the previous two scripts, 'get_aruco_coord.py' and 'transformation_matrix.py'**
<br>


# Obstacle avoidance

Detect and avoid obstacles are important features for a mobile robot to navigate in an unknown environment. First step is detecting the obstacles where simple object detection or deep learning can be used. Second step is the path planning to reach the target.

We have used lidar to avoid obstacle in our algorithm.

## 1. Obstacle detection

Detection of an obstacle is an improtant part of any self-driving robot. Lidar is a very reliable device to calculate distance from nearby field within its detectable range. The lidar in turrtlebot3 has a range of 120 ~ 3,500mm.

## 2. The algorithm developed

We have read the lidar data on specific angles which are 0 degree, 15 degrees and 345 degrees.

0 degree is the front of the turtlebot3 and it measures in an anticlockwise manner.
The data we get from turtlebot3 lidar is continious but not proper reliable as the data is ambiguous. While scanning the surrounding, the lidar sends lots of zeros and it makes the detection difficult in real time.

Different turtlebots have been tried to overcome this problem but it seems that they are made out of same mother.

A threshold of 300mm is kept for avoidance of obstacles which is, if any object is within the range of 300mm of the robot at 0, 15, 345 degrees, the robot will make it's provided direction other than the calculated direction from the transformation matrix.




# Conclusion

Controlling a robot in an unknown environment add more challenge to the control system and to the detection. Detecting the robot will not be as easy as while using the aruco marker. Moving the robot in a rugged terrain needs a robust control system that takes in consideration the tough surface and the sliding of the wheels.

The project allowed us to take on hand several important robotic skills, image processing, visual servoing, path planning, interpretation of the result, frame transformation, and soft skills as well.
   
# References
[ROS TurtleBot3 Basics](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)\
[ROS Wiki](http://wiki.ros.org/noetic)\
[ROS Basics course](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/ros-perception-in-5-days/)\
[ROS perception course](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/ros-perception-in-5-days/)
