# Kinematics in MATLAB

## 1. Introduction

In this README, the kinematic model implemented in MATLAB for a cable robot that controls an end effector in a 2D plane is explained, which is implemented in the file [cinematic_model.m](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/matlab/cinematic_model.m).

It's important to mention that cable robots are mechanical systems in which the end effector is manipulated by cables instead of rigid links, offering advantages such as the ability to operate in large workspaces and having a high strength-to-weight ratio.

## 2. System Description

The system is mainly composed of 4 major elements:

* **Fixed "Blackboard-Type" Structure**: A 100 x 100 cm (1 x 1 m) rectangular frame.
* **Pulleys**: Located at the top corners of the frame, where M1 is in the upper left corner at position (0, 100), while M2 is in the upper right corner at position (100, 100).
* **End Effector**: A 10 x 10 cm rectangular object.
* **Cables**: Connect each of the pulleys to each of the top corners of the end effector, where L1 is the cable holding the left area, while L2 is the cable holding the right area.

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/matlab_kinematic_model.png?raw=true">
</p>

## 3. Direct Kinematic Model

The forward kinematic model calculates the position of the end effector given the cable lengths.

The end effector is a square centered at position (x_effector, y_effector). Therefore, the following formulas are used to calculate the position of each of the end effector's top corners (those to which the cables are attached):

```matlab
% Top left corner (x1, y1)
x1 = x_efector - (largo_efector / 2);
y1 = y_efector + (alto_efector / 2);
```

```matlab
% Top right corner (x2, y2)
x2 = x_efector + (largo_efector / 2);
y2 = y_efector + (alto_efector / 2);
```

## 4. Calculating Cable Lengths

The length of each of the cables L1 (holds the left area) and L2 (holds the right area) can be calculated using the Euclidean distance (direct application of the Pythagorean theorem) between the position of each of the pulleys (M1x, M1y) and (M2x, M2y) and each of the upper corners of the end effector (x1, y1) and (x2, y2) as follows:

```matlab
L1 = sqrt((x1 - M1x)^2 + (y1 - M1y)^2);
```

```matlab
L2 = sqrt((x2 - M2x)^2 + (y2 - M2y)^2);
```

## 5. Calculating Cable Angles

Angles q1 and q2 represent the orientation of each cable with respect to the vertical, and are calculated as follows:

```matlab
% Cable Angle / Corner Left Effector
q1 = -rad2deg(atan((x1 - M1x) / (y1 - M1y)));
```

```matlab
% Cable Angle / Right Effector Corner
q2 = rad2deg(atan((x2 - M2x) / (y2 - M2y)));
```

The atan() function calculates the arctangent of the quotient between the horizontal and vertical components of the cable vector, where the sign in q1 sets the angle convention.

For example, to calculate the angle formed by cable L1, follow four steps:

* The cable vector is (x1 - M1x, y1 - M1y) = (x1 - 0, y1 - 100).
* The angle q1 is formed between this vector and the vertical.
* tan(q1) = x1 / (y1 - 100)
* Therefore, q1 = atan(x1 / (y1 - 100))

The process is identical for L2, with the only difference being that the motor/pulley holding L2 is at position (M2x, M2y) = (100, 100).

## 6. Inverse Kinematic Model

Although the inverse kinematic model is not directly implemented in the code, it could be used to calculate the end-effector positions knowing the cable lengths, which would involve solving a system of nonlinear equations:

```matlab
L1^2 = (x1 - largo_efector / 2)^2 + (y1 + alto_efector / 2 - 100)^2
```

```matlab
L2^2 = (x2 + largo_efector / 2 - 100)^2 + (y2 + alto_efector / 2 - 100)^2
```

## 7. Limit Check

And finally, the code includes a series of checks to ensure that the effector remains within the workspace and that the effector does not collide with the edges of the structure:

```matlab
% LIMIT VERIFICATION
error = false;
if x_efector >= (largo_plano - (largo_efector / 2))
    fprintf("FUERA DEL LÍMITE en x = %.2f\n", x_efector);
    error = true;
elseif x_efector <= (largo_efector / 2)
    fprintf("FUERA DEL LÍMITE en x = %.2f\n", x_efector);
    error = true;
end
if y_efector >= (alto_plano - (alto_efector / 2))
    fprintf("FUERA DEL LÍMITE en y = %.2f\n", y_efector);
    error = true;
elseif y_efector <= (alto_efector / 2)
    fprintf("FUERA DEL LÍMITE en y = %.2f\n", y_efector);
    error = true;
end
if error
    return;
end
```

## 8. Conclusions

This kinematic model implements the fundamental geometric relationships between the pulleys and the end effector of a cable-driven robot in a plane. The derived equations are based on simple Euclidean geometry but provide a solid foundation for position control of the system.

Possible future extensions include implementing the inverse kinematic model, adding dynamics to the system, considering cable elasticity, and extending the system to three dimensions.
