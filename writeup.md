## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./images/KukaKR210.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This document.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

In first place I sketch the robot arm in its zero configuration identifying DH table parameters:

![alt text][image1]

Next we create the DH table having into account the xacro file and the schema.

i | alpha i-1 | a i-1 | di | qi
--- | --- | --- | --- | ---
1: T0_1| 0 | 0 | 0.75 | 0
2: T1_2| -90 | 0.35 | 0 | -90
3: T2_3| 0 | 1.25 | 0 | 0
4: T3_4| -90 | -0.054 | 1.5 | 0
5: T4_5| 90 | 0 | 0 | 0
6: T5_6| -90 | 0 | 0 | 0
G: T6_G| 0 | 0 | 0.303 | 0

Alpha angles are defined between z axis. For example, Z0 and Z1 are coincident and the angle is 0  degrees. Between Z1 and Z2 there is a  rect angle in negative sense, -90  degrees. Z2 and Z3 are paralell 0 degrees, and so on.

Diastancies A and D come from the urdf file. A1 can be found in the x position o of the definition of joint 2. A2 and A3, as they are perpendicular to the x axis of the origin, can be found in the z coordinate of joint3 and joint4. 
D1 can be found in joint1 and joint2 adding z coordinates over the base: 0.33 + 0.42 = 0.75. D4 can be found adding x positions of joints 4 and five (distance from joint 3) 0.54 + 0.96 = 1.5. DG or D7 can be obtained from the x position of joint 6 plus the x position of the gripper_joint: 0.193 + 0.11 = 0.303.

Angles qi are positioned at 0 except for q2 which is a -90.


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Transformation matrices are calculated using Python (the code included later in IK_server.py). The results are:

T0_1 = Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

T1_2 = Matrix([
[sin(q2),  cos(q2), 0, 0.35],
[      0,        0, 1,    0],
[cos(q2), -sin(q2), 0,    0],
[      0,        0, 0,    1]])

T2_3 = Matrix([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,    0],
[      0,        0, 0,    1]])

T3_4 = Matrix([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]])

T4_5 = Matrix([
[cos(q5), -sin(q5),  0, 0],
[      0,        0, -1, 0],
[sin(q5),  cos(q5),  0, 0],
[      0,        0,  0, 1]])

T5_6 = Matrix([
[ cos(q6), -sin(q6), 0, 0],
[       0,        0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[       0,        0, 0, 1]])

T6_G = Matrix([
[1, 0, 0,     0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]])


T0_G = Matrix([
[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],
[((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],
[                                                               -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                               -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                                                                                           0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                            1]])



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

I get wrist position from end position using vector l of the rotation matrix because end efector z axe is base x axe:
            lx = cos(roll)*cos(pitch)
            ly = sin(roll)*cos(pitch)
            lz = -sin(pitch)

            wx = px-(gripper_length)*lx
            wy = py-(gripper_length)*ly
            wz = pz-(gripper_length)*lz

Using trigonometry I get the angles q1, q2 and q3 and I transform them to apply them to the arm:
            q1 = atan2(wy,wx)
            q2= atan2(wz-z_2,sqrt((wx-x_2)**2 + (wy-y_2)**2)) + acos((a2**2.0 - d3**2.0 + dist1**2.0)/(2.0*a2*dist1)) 
            q3 = acos((a2**2.0 + d3**2.0 - dist1**2.0)/(2.0*a2*d3))

            q2 = (q2 - pi/2.0) * -1.0
            q3 = (q3 - pi/2.0) * -1.0

For the wrist angles I have used a geometrical calculation which is faster because it doesn't use FK matrices:
- Calculate roll and pitch in wrist coordinates space:
            roll2 = roll - q1
            pitch2 = pitch - (q2 + q3)
- End efector projection in axes:
            dgry = 0.303*cos(pitch2)*sin(roll2)
            dgrz = 0.303*sin(pitch2)
- Calculate angles with trigonometry:
            q4 = atan2(dgry, dgrz)
            q5 = atan2(sqrt(dgry**2+dgrz**2),cos(pitch2)*cos(roll2))
            if pitch>-pi/2 and pitch<pi/2:
                q6 = yaw - q4
            else:
                q6 = pi - (yaw - q4)

Forward kinematics is included commented for evaluation purposes.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


I have implemented IK follwing Udacity recommendations except for q4, q5 and q6. I have used a geometrical approach for them which is faster to calculate than FK matrices.
The results are good. Pieces go to the bin the majority of times.
Movements stop sometimes because collision between links 4 and 6 but I don't know why because I can't see that collition. However after the stops the arm ends in a good position.

For a future implementation there is the posibility to implement an error showing function and compare my aproach with the rotation matrix method for the wrist angles.

