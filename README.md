# Robotic Surgery tool implementation
This project, part of the "Introduction to Robotics" course, focuses on implementing a robotic surgery tool using a 6-DOF (Degrees of Freedom) anthropomorphic robotic arm. The objective is to implement translational velocities in the end-effector under an RCM(Remote Center of Motion) constraint, specifically for simulating the incision, a circular ring was used and a tool was attached to  the robot-arm and the goal was to keep the incision point fixed at the entry point of the incision, which was considered to be the center of the ring, while being able to implement velocities at the end-effector. 

The main problem was to decouple the joint velocity into two perpendicular components, one related to the velocity in the free-space  and the velocity in the constrained-space so that we can achieve better results in the expirements. For the theoretical basis of the project we were relied on the work of  Kastritsi Theodora, "Shared control with haptic feedback for robotic-assisted minimally invasive surgery".



## Expirements
The robot-arm was tested in implementing rotational velocity around the z axis of the end-effector(axis of the tool shaft) and translational velocities around x,y,z axis of the end-effector. Besides the RCM constraint, a constraint for keeping the tool away from the incision walls was added, but it was mainly used for stopping the arm from hitting the walls because the system was underactuated and could not complete secondary tasks in the null space.

## Results
The following sections include videos of the implementation, followed by graphs of the minimum distance and RCM constraint.


### Translational velocity around z-axis
https://github.com/user-attachments/assets/319a279b-878e-4830-8cf8-a6c439822d0e

![image](https://github.com/user-attachments/assets/81235272-2064-41db-9185-ea12d9c641bd)
RCM constraint
![image](https://github.com/user-attachments/assets/bae86495-2d88-47ec-8555-4497d8fb798e)
Minimum distance
### Translational velocity around x-axis
https://github.com/user-attachments/assets/1d0eca92-6c3d-4e65-b7f3-3b19860cd0fb

![image](https://github.com/user-attachments/assets/8bccee6f-64af-49c4-bda7-e919e6a1aca2)
RCM constraint
![image](https://github.com/user-attachments/assets/4231e160-b6f0-4c11-a6e3-90b101f1ae5c)
Minimum distance

### Translational velocity around y-axis
https://github.com/user-attachments/assets/bedba194-ba6a-4628-bbb2-9610c99ffc30

![image](https://github.com/user-attachments/assets/f5213cbd-f4cc-4ee8-a1c5-beac2f5d9061)
RCM constraint
![image](https://github.com/user-attachments/assets/afcb9cdf-8efd-4876-b991-936583f025dc)
Minimum distance
### Rotational velocity around z-axis
https://github.com/user-attachments/assets/81738892-d344-442f-b1db-f081fdc2293e

![image](https://github.com/user-attachments/assets/d3e5a036-ee7e-40ac-b0d2-a7306ceb00a0)
RCM constraint
![image](https://github.com/user-attachments/assets/8c1062cb-b77a-4d22-b541-aa8d5c9b951e)
Minimum distance

## Result Evaluation
The tool despite not remaining completelly still at the entry point of the incision((this is probably caused by imperfections in the robot-arm's construction) it does not deviate much from it and  returns to it.The end-effector velocities are also achieved quite well.

As expected, the results have a deviation from reality, especially concerning the minimum distance, since there is a great uncertainty in the exact dimensions of each joint which was not taken into account. Even though there were cases where the arm collided with the incision this is not reflected in the graphs. However, they do correctly capture the tendency of the result. 

On the other hand, the RCM constraint graphs are more precise and clearly show the tendency of the graph to return to zero even when it deviates for a while as this is the primary objective.
## Acknowledgments
Kastritsi Theodora, "Shared control with haptic feedback for robotic-assisted minimally invasive surgery"
