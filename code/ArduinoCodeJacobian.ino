#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#define PROVIDE_ONLY_LINEAR_MOVEMENT
#include "ServoEasing.hpp"
//Inverse Kinematics


ros::NodeHandle  nh;

ServoEasing s1;
ServoEasing s2;
ServoEasing s3;
ServoEasing s4;
ServoEasing s5;
ServoEasing s6;

int angle1_mapping(float angle){

  return constrain(round((angle+53)/262*180),0,180);
}

int angle2_mapping(float angle){

  return constrain(round((angle+53)/262*180),55,180);
}

int angle3_mapping(float angle){

  return constrain(round((angle+53)/262*180),55,180);
}

int angle4_mapping(float angle){

  return constrain(round((angle+53)/262*180),0,180);
}

int angle5_mapping(float angle){

  return constrain(round((angle+53)/262*180),0,120);
}

int angle6_mapping(float angle){

  return constrain(round((angle+53)/262*180), 0,180);

}

int s1_angle=angle1_mapping(90);
int s2_angle=angle2_mapping(90);
int s3_angle=angle3_mapping(90);
int s4_angle=angle4_mapping(90);
int s5_angle=angle5_mapping(90);
int s6_angle=angle6_mapping(90);




void servo_cb(const std_msgs::Float64MultiArray & msg){
  
  s1_angle=angle1_mapping(90+msg.data[0]);
  s2_angle=angle2_mapping(180-msg.data[1]);
  s3_angle=angle3_mapping(180-msg.data[2]);
  s4_angle=angle4_mapping(90+msg.data[3]);
  s5_angle=angle5_mapping(90+msg.data[4]);
  s6_angle=angle6_mapping(msg.data[5]);
  s1.write(s1_angle);
  s2.write(s2_angle);
  s3.write(s3_angle);
  s4.write(s4_angle);
  s5.write(s5_angle);
  s6.write(s6_angle);
  //s1.setEaseTo(s1_angle);
  //s2.setEaseTo(s2_angle);
  //s3.setEaseTo(s3_angle);
  //s4.setEaseTo(s4_angle);
  //s5.setEaseTo(s5_angle);
  //s6.startEaseTo(s6_angle);
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("/joint_states_pub", servo_cb);

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  
  //setSpeedForAllServos(40);

  s1.attach(7,90);
  delay(1);
   
  s2.attach(6,90);
  delay(1);
  
  s3.attach(5,90);
  delay(1);

  s4.attach(4,90);
  delay(1);

  s5.attach(3,90);
  delay(1);

  s6.attach(2,90);
  delay(1);
  
}

void loop() {
  
  //s1.setEaseTo(s1_angle);
  //s2.setEaseTo(s2_angle);
  //s3.setEaseTo(s3_angle);
  //s4.setEaseTo(s4_angle);
  //s5.setEaseTo(s5_angle);
  //s6.startEaseTo(s6_angle);

  //s1.write(s1_angle);
  //s2.write(s2_angle);
  //s3.write(s3_angle);
  //s4.write(s4_angle);
  //s5.write(s5_angle);
  //s6.write(s6_angle);

  
  //while (ServoEasing::areInterruptsActive()) {


   //}
  nh.spinOnce();
  
  //delay(10);

}