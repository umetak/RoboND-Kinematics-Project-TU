## Project: Kinematics Pick & Place

[//]: # (Image References)
[image1]: ./misc_images/fig1.png
[image2]: ./misc_images/fig2.png


---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The Kuka KR210 DH parameters were obtained by measuring data in the forward_kinematics demo.

![alt text][image1]

The DH parameter table is shown below.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Parameter | Name | Definition   
--- | --- | ---
alpha(i-1) | Twist angle | Angle from z(i-1) axis to z(i) axis measured along the x(i-1) axis
a(i-1) | Link length | Distance from z(i-1) axis to z(i) axis measured along the x(i-1) axis
d(i) | Link offset | Distance from x(i-1) axis to x(i) axis measured along the z(i) axis
theta(i) | Joint variable | Angle from x(i-1) axis to x(i) axis measured along the z(i) axis

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
```python
# Modified DH parameters KR210 forward kinematics sections
DH_table = {alpha0:       0, a0:      0, d1:  0.75, q1:           q1,
            alpha1: -pi/2.0, a1:   0.35, d2:     0, q2: -pi/2.0 + q2,
            alpha2:       0, a2:   1.25, d3:     0, q3:           q3,
            alpha3: -pi/2.0, a3: -0.054, d4:   1.5, q4:           q4,
            alpha4:  pi/2.0, a4:      0, d5:     0, q5:           q5,
            alpha5: -pi/2.0, a5:      0, d6:     0, q6:           q6,
            alpha6:       0, a6:      0, d7: 0.303, q7:            0}

# Define Modified DH Transformation matrix
def TF_matrix(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                 [ cos(alpha)*sin(q), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                 [ sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  cos(alpha)*d],
                 [                 0,                 0,           0,             1]])
    return TF

# Create individual transformation matrices
T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(DH_table)
T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(DH_table)
T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(DH_table)
T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(DH_table)
T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(DH_table)
T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(DH_table)
T6_EE = TF_matrix(alpha6, a6, d7, q7).subs(DH_table)

T0EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
T0_3 = T0_1 * T1_2 * T2_3
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

####Inverse Position Kinematics
We have the end-effector yaw-pitch-roll angles, therefore the end-effector center position can be computed.
```python
def Rotation_matrix(r, p, y):

    Rot_x = Matrix([[   1,      0,       0],
                    [   0, cos(r), -sin(r)],
                    [   0, sin(r),  cos(r)]]) # Roll

    Rot_y = Matrix([[ cos(p),  0, sin(p)],
                    [      0,  1,      0],
                    [-sin(p),  0, cos(p)]]) # Pitch

    Rot_z = Matrix([[ cos(y), -sin(y),  0],
                    [ sin(y),  cos(y),  0],
                    [      0,       0,  1]]) # Yaw

    ROT_EE = Rot_z * Rot_y * Rot_x
    Rot_corr = Rot_z.subs(y, np.pi) * Rot_y.subs(p, -np.pi/2.0)
    ROT_EE = ROT_EE * Rot_corr

    return ROT_EE

ROT_EE = Rotation_matrix(r, p, y).subs({'r': roll, 'p': pitch, 'y': yaw})

EE = Matrix([[px], [py], [pz]])
WC = EE - 0.303 * ROT_EE[:,2]

# Calculate joint angles using Geometric IK method
theta1 = atan2(WC[1], WC[0])

side_a = 1.501
side_b = sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2] - 0.75)**2)
side_c = 1.25

angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))

theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) - 0.35)
theta3 = pi/2 -(angle_b + 0.036)
```

####Inverse Orientation Kinematics
By using the rotation transformation matrix we can compute joint angles with LU decomposition.
```python
R0_3 = T0_3[:3,:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv('LU') * ROT_EE

theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

I examined my implementation through try and errors. And I finally figured out that adding condition clauses showed better performance.

```python
if sin(theta5) > 0:
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
else:
    theta4 = atan2(-R3_6[2,2], R3_6[0,2])
    theta6 = atan2(R3_6[1,1], -R3_6[1,0])

if x >= len(req.poses):
    theta5 = theta5_temp
    theta6 = theta6_temp

theta5_temp = theta5
theta6_temp = theta6
```

Screenshot from my result:
![alt text][image2]
