#!/usr/bin/env python
import rospy
import numpy as np
from math import cos, sin, pi, atan2, acos, sqrt
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose
from numpy.linalg import inv, det, norm, pinv
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
class Robot_Manipulator_Controller:

    def __init__(self):

        self.d1 = 13
        self.a2 = 8.5
        self.d4 = 8
        self.d6= 12

        self.dz=np.array([[-90 , 90],\
                          [0 , 125],\
                          [0 , 125],\
                          [-90 , 90],\
                          [-90 , 30],\
                          [0 , 180]])


        self.alpha1 = pi/2
        self.alpha2 = 0
        self.alpha3 = pi/2
        self.alpha4= -pi/2
        self.alpha5 = pi/2
        self.alpha6 = 0

        self.angle_msg = Float64MultiArray()
        self.angle_msg.layout.dim.append(MultiArrayDimension())
        self.angle_msg.layout.dim[0].size = 6
        self.angle_msg.layout.dim[0].stride = 6
        self.angle_msg.layout.data_offset = 0

        self.starting_point=[0,75,20,45,-30,45]
        self.translation_d=[14.44 ,-10.653, 15.187]
        self.k=np.diag([1,1,1,1,0.1,0.1])
        self.circle=np.array([[14.44,-10.653,15]]).T
        self.radius=4.5
        self.circle_orient=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        self.warning=False

        print(self.circle_orient)
   

        self.angle_msg.data = self.starting_point
        self.joint_angvel=[0,0,0,0,0,0]
        self.j_pub = rospy.Publisher('joint_states_pub',Float64MultiArray,queue_size=100)
        self.rate = rospy.Rate(10)
        self.publish()



    def publish(self):
     np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

     self.joint_angpos = [-1.346, 0.606, 0.725 ,1.489, -1.677, -0.165]


     vu=np.array([[0,0,0,1]]).T
     ue=np.array([[0,0,0,0,0,0]]).T
     T = self.tf_A06(self.joint_angpos)
     Pc=T[0:3, 3]

     while not rospy.is_shutdown():


                A06 = self.tf_A06(self.joint_angpos)
                J = self.compute_jacobian(self.joint_angpos)



                Nt = A06[0:3, 2]
                Bc = A06[0:3, 0:2]
                Rt = A06[0:3, 0:3]
                Pt = A06[0:3, 3]


                xc=Bc.T@(Pt-Pc)
                Ax = Bc.T @ np.hstack((np.eye(3), self.skew_from_vector(Pt - Pc)))
                A = Ax @ J

                Zx = np.block([[Nt.T, np.zeros((1,3))], [Rt.T @ self.skew_from_vector(Pt - Pc), Rt.T]])
                Z = (pinv(J) @ Zx.T).T


                N = (Z.T).T
                S = np.hstack((pinv(A), N.T))
                qdot = pinv(J) @ ue



                xcdot = A @ qdot


                ue = (Zx.T @ vu + J @ pinv(A) @ xcdot)
                qdot = S @ np.hstack((xcdot.T, vu.T)).T
                self.Li(self.joint_angpos)
                self.mindist(self.joint_angpos, self.circle, self.radius, self.circle_orient)
                if(self.warning==True):
                    break


                self.joint_angvel=np.copy(qdot)
                




                dt=0.1

                # Integration
                self.joint_angpos[0] = self.joint_angpos[0] + dt * self.joint_angvel[0]
                self.joint_angpos[1] = self.joint_angpos[1] + dt * self.joint_angvel[1]
                self.joint_angpos[2] = self.joint_angpos[2] + dt * self.joint_angvel[2]
                self.joint_angpos[3] = self.joint_angpos[3] + dt * self.joint_angvel[3]
                self.joint_angpos[4] = self.joint_angpos[4] + dt * self.joint_angvel[4]
                self.joint_angpos[5] = self.joint_angpos[5] + dt * self.joint_angvel[5]

                self.angle_msg.data=np.degrees(self.joint_angpos)

                self.j_pub.publish(self.angle_msg)
                A06 = self.tf_A06(self.joint_angpos)
                T06=A06[0:3,3]
                print('Translation=',T06)

                self.rate.sleep()
    def mindist(self, angle_pos, obstacle, radius, R):
        obstacle = obstacle.reshape((3,1))
        distances=[]
   

        positions=[self.tf_A01(angle_pos)[:3, 3], self.tf_A02(angle_pos)[:3, 3],
                self.tf_A03(angle_pos)[:3, 3], self.tf_A04(angle_pos)[:3, 3],
                self.tf_A05(angle_pos)[:3, 3], self.tf_A06(angle_pos)[:3, 3]]


        initial_guess = np.array([0, np.pi])
        for i in range(len(positions)-1):
            pi=positions[i]
            pnext=positions[i+1]
            p=pnext-pi
            if np.linalg.norm(p)>1e-3:
                args=(pi,pnext,obstacle,radius,R)
                upper_limit = np.linalg.norm(pnext - pi)

                # Define the bounds
                bounds = [(0, upper_limit), (0, 2*np.pi)]
                result = minimize(self.objective_function, initial_guess, args=args,  bounds=bounds)

                distances.append(result.fun)


        if min(distances)<3:
            print("warning dist")
            self.warning=True

        return min(distances)






    def objective_function(self,x,pi,pf,circle_center, radius,R):
        s=x[0]
        t=x[1]
        pi=pi.reshape((3,1))
        pf=pf.reshape((3,1))

        d=pi+s*(pf-pi)/np.linalg.norm(pf-pi) - circle_center -radius*(np.cos(t)*R[:3,0]+np.sin(t)*R[:3,1])

        return np.linalg.norm(d)
    
    def singularity(self,angle_pos):
        J = self.compute_jacobian(angle_pos)

        return np.linalg.det(J@J.T)**0.5
   
    def L(self,theta):
     Li = np.zeros(len(theta))
     for i in range(len(theta)):
        zi = (self.dz[i, 1] + self.dz[i, 0]) / 2
        Li[i] = ((theta[i] - zi) / (self.dz[i, 1] - self.dz[i, 0]))**2
     if np.max(Li)>1:
         print("warning")
         self.warning=True


     w=sum(Li)/12
     return w

    def turn_off(self):
        pass

    def tf_A01(self, r_joints_array):
        i=0
        tf = np.array([[cos(r_joints_array[i]) , -sin(r_joints_array[i])*cos(self.alpha1) , sin(r_joints_array[i])*sin(self.alpha1)  , 0*cos(r_joints_array[i])],\
                        [sin(r_joints_array[i]) , cos(r_joints_array[i])*cos(self.alpha1), -cos(r_joints_array[i])*sin(self.alpha1)  , 0*sin(r_joints_array[i])],\
                        [0 , sin(self.alpha1) , cos(self.alpha1) , self.d1 ],\
                        [0 , 0 , 0 , 1]])
        return tf

    def tf_A02(self, r_joints_array):
        i=1
        tf_A12 = np.array([[cos(r_joints_array[i]) , -sin(r_joints_array[i])*cos(self.alpha2) , sin(r_joints_array[i])*sin(self.alpha2)  , self.a2 *cos(r_joints_array[i])],\
                        [sin(r_joints_array[i]) , cos(r_joints_array[i])*cos(self.alpha2), -cos(r_joints_array[i])*sin(self.alpha2)  , self.a2 *sin(r_joints_array[i])],\
                        [0 , sin(self.alpha2) , cos(self.alpha2) , 0 ],\
                        [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):
        i=2
        tf_A23 = np.array([[cos(r_joints_array[i]) , -sin(r_joints_array[i])*cos(self.alpha3) , sin(r_joints_array[i])*sin(self.alpha3)  , 0*cos(r_joints_array[i])],\
                        [sin(r_joints_array[i]) , cos(r_joints_array[i])*cos(self.alpha3), -cos(r_joints_array[i])*sin(self.alpha3)  , 0 *sin(r_joints_array[i])],\
                        [0 , sin(self.alpha3) , cos(self.alpha3) ,  0 ],\
                        [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        i=3
        tf_A34 = np.array([[cos(r_joints_array[i]) , -sin(r_joints_array[i])*cos(self.alpha4) , sin(r_joints_array[i])*sin(self.alpha4)  , 0*cos(r_joints_array[i])],\
                        [sin(r_joints_array[i]) , cos(r_joints_array[i])*cos(self.alpha4), -cos(r_joints_array[i])*sin(self.alpha4)  , 0 *sin(r_joints_array[i])],\
                        [0 , sin(self.alpha4) , cos(self.alpha4) ,  self.d4 ],\
                        [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):
        i=4
        tf_A45 = np.array([[cos(r_joints_array[i]) , -sin(r_joints_array[i])*cos(self.alpha5) , sin(r_joints_array[i])*sin(self.alpha5)  , 0*cos(r_joints_array[i])],\
                        [sin(r_joints_array[i]) , cos(r_joints_array[i])*cos(self.alpha5), -cos(r_joints_array[i])*sin(self.alpha5)  , 0 *sin(r_joints_array[i])],\
                        [0 , sin(self.alpha5) , cos(self.alpha5) ,  0 ],\
                        [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        i=5
        tf_A56 = np.array([[cos(r_joints_array[i]) , -sin(r_joints_array[i])*cos(self.alpha6) , sin(r_joints_array[i])*sin(self.alpha6)  , 0*cos(r_joints_array[i])],\
                        [sin(r_joints_array[i]) , cos(r_joints_array[i])*cos(self.alpha6), -cos(r_joints_array[i])*sin(self.alpha6)  , 0 *sin(r_joints_array[i])],\
                        [0 , sin(self.alpha6) , cos(self.alpha6) ,  self.d6 ],\
                        [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def skew_from_vector(self,a):
       return np.cross(a, np.identity(a.shape[0]) * -1)

    def compute_jacobian(self, r_joints_array):



        A01 = self.tf_A01(r_joints_array)
        A02 = self.tf_A02(r_joints_array)
        A03 = self.tf_A03(r_joints_array)
        A04 = self.tf_A04(r_joints_array)
        A05 = self.tf_A05(r_joints_array)
        A06 = self.tf_A06(r_joints_array)
        p=A06[0:3,3] # end-effector position
        p=np.transpose(p)
        z = np.array([0, 0, 1])
        z0=z
        z1 = A01[0:3,2]
        z1=np.transpose(z1)
        z2 = A02[0:3,2]
        z2=np.transpose(z2)
        z3 = A03[0:3,2]
        z3=np.transpose(z3)
        z4 = A04[0:3,2]
        z4=np.transpose(z4)
        z5 = A05[0:3,2]
        z5=np.transpose(z5)

        pz = np.array([0, 0, 0])

        p0=pz
        p1 = A01[0:3,3]
        p1=np.transpose(p1)
        p2 = A02[0:3,3]
        p2=np.transpose(p2)
        p3 = A03[0:3,3]
        p3=np.transpose(p3)
        p4 = A04[0:3,3]
        p4=np.transpose(p4)
        p5 = A05[0:3,3]
        p5=np.transpose(p5)



        J1 = np.cross(z0,p-p0)

        J2 = np.cross(z1,p-p1)
        J3 = np.cross(z2,p-p2)
        J4 = np.cross(z3,p-p3)
        J5 = np.cross(z4,p-p4)
        J6 = np.cross(z5,p-p5)


        J_tmp1=np.c_[np.transpose(J1), np.transpose(J2),np.transpose(J3),np.transpose(J4),np.transpose(J5),np.transpose(J6)]

        J_tmp2=np.c_[np.transpose(z0),np.transpose(z1),np.transpose(z2),np.transpose(z3),np.transpose(z4),np.transpose(z5)]

        J=np.r_[J_tmp1,J_tmp2]


        return J

def controller():
    # Starts a new node
    rospy.init_node('control_node')

    controller = Robot_Manipulator_Controller()
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass