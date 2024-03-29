![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image004.png)

<<<<<<< HEAD
# Table of Contents:
=======
# TABLES OF CONTENTS:
>>>>>>> dbe6b6fefe41912e8cfddf0c2a5047211ebc8fa1


 1. [Introduction](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#introduction)

 2. [Camera Calibration](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#camera-calibration)

 3. [Detection and pose estimation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#detection-and-pose-estimation)

   1) [ Approximation 2D without pose estimation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-approximation-2d-without-pose-estimation)

   2) [3D pose estimation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-3d-pose-estimation)

 4. [Frame transformation](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#frame-transformation)

 5. [Control system](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#control-system)

    * [Simulation to test the control system](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#control-system)

 6. [Integration of ROS ](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#integration-of-ros)

    1) [ What is ROS: ](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-what-is-ros)

    2) [ What is the publisher and Subscriber](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-what-is-the-publisher-and-subscriber)

    3) [Launch the Robot](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#3-launch-the-robot)

    4) [Launch the Camera](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#4-launch-the-camera)

 7. [Obstacle avoidance](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#obstacle-avoidance)

    1) [Obstacle detection](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#1-obstacle-detection)

    2) [The algorithm developed](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#2-the-algorithm-developed)

    3) [Similar RRT algorithm](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#3-similar-rrt-algorithm)

 8. [Conclusion](https://github.com/zainbinsumait/Multi_Sensor_Fusion_and_Tracking#conclusion)




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

![](https://user-images.githubusercontent.com/76461363/206849013-a1c9ff25-d76f-47aa-b780-a7aac427da26.png))

## 3.    Launch the Robot:

The robot is already delivered with all the necessary ROS packages, otherwise you can easily get them by flowing a good tutorial made by ‘ROBOTIS’ [https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

To launch the robot in the terminal, write these commands:

![Text Box: Roscore
Ssh ubuntu@192.168.0.200 “the password is napelturbot”
roslaunch turtlebot3_bringup turtlebot3_robot. launch](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image073.png)

Till now we started these topics on our robot:

![Une image contenant texte
Description générée automatiquement](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image075.png)

## 4.    Launch the Camera:

To publish our code in the robot, we must get the topics of the camera first; for that there is already Ros packages of the Ueye-camera by installing them from: [https://github.com/anqixu/ueye_cam](https://github.com/anqixu/ueye_cam)

![Text Box: Roslaunch ueye_cam rgb8.launch](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image076.png)

## Move the robot

This step is to publish the speed calculated by publishing it on the robot’s topic “cmd_vel”. A servoing closed loop that gets the real time image, calculates the velocity, and sends it to the robot by a frequency of 50 Hz. The robot receives the instruction and goes to the target. The robot stops when the distance between the current and the target is about 40 pixels or (40/f)*Z = 5.6 cm (where f is the focal length and Z is the distance to the camera). And for the parking we added another condition to adjust the angle θ, so the robot has the same orientation as the target.

![Text
Description automatically generated](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image078.png)

# Obstacle avoidance

Detect and avoid obstacles are important features for a mobile robot to navigate in an unknown environment. First step is detecting the obstacles where simple object detection or deep learning can be used. Second step is the path planning to reach the target.

Numerous of algorithm exist to determine the shortest path like A* and RRT. We chose to develop our own method to avoid obstacles as well as RRT algorithm.

## 1.    Obstacle detection

Detect an object considered as an obstacle needs a computer vision. Depending on the desired result, it can be a simple or a complex detection from shape or colour detection to a deep learning model to detect any object that can be considered as an obstacle. In this project we chose a simple colour detection with a red colour mask to extrait the red objects from the image.

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image081.png)

## 2.    The algorithm developed

The input image is divided into squares with the same size as the robot in pixels, then we consider the square with red colour pixels as an obstacle square which can’t be added to the path. Next, for each box a cost is calculated by the following equations:

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image083.png)

Where d is the distance from the centre of the square to the target. 

n = ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image085.png) if the square is an obstacle neighbour and n = 0 if not.

This equation returns a cost with an inverse relationship to the distance to the target with a bigger value to the obstacle’s neighbour squares, this gives a safety margin to completely avoid the obstacle.

The algorithm steps:

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image087.png)

Applying this algorithm in real life gives us the result in the following figure. The path is as far as it can be from the obstacles to have a safe margin, however, it’s not exactly the optimist way. The algorithm can be improved to calculate the total cost of the path and then try another path as what other algorithms do (A star and RRT).  

The path points are then sent to the control system so that the robot reach each point as it’s finale target. Once it reached to the first point it goes to the second until the end of the path.

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image089.png)

## 3.    Similar RRT algorithm

Rapidly exploring Random Tree is an algorithm used in path planning to create a safety path for a trajectory.

Like the RRT process algorithm we try to describe a path that the robot can follow safety to join the target position avoiding the different obstacles.

The RRT algorithm follows the step below.

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image090.png)![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image091.png)![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image093.png)

In first time, around the start point we create a point by using a random vector generate by the goal position and the concept of nearest neighbour. Then we check if this point is not at the position of the obstacle and if it respects the condition due to its distance to the goal position. If this constraint is done, we add it in the path list and restart the process with this point as the start point now. In a small simulation we fixe an obstacle and create the path

![Chart, scatter chart
Description automatically generated](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image095.png).

In the image in green we have different points created by the algorithm. When the point is close to the obstacle, it creates a new point by respecting the constraint due to its distance from him until the robot arrives at the goal.

Now we apply it on an image. First with an algorithm for colour detection we detect the obstacle, and we extract its position. In this case the position is extracted in pixel.

                  ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image097.png)                  ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image099.png)

With this position and the robot and target position we can apply the algorithm to have the path that the robot can follow.

![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image101.png)              ![](file:///C:/Users/zain/AppData/Local/Temp/msohtmlclip1/01/clip_image103.png)

# Conclusion

Controlling a robot in an unknown environment add more challenge to the control system and to the detection. Detecting the robot will not be as easy as while using the aruco marker. Moving the robot in a rugged terrain needs a robust control system that takes in consideration the tough surface and the sliding of the wheels. We can use a deep learning algorithm to detect the robot and the target. Using a CNN models to detect the robot without a marker in almost all the situation depending on how strong the model is.

The project allowed us to take on hand several important robotic skills, image processing, visual servoing, path planning, interpretation of the result, frame transformation, and soft skills as well.
