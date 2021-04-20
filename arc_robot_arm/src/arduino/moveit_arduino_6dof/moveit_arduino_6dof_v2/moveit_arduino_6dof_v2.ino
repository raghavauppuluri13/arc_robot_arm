
/*
 * rosserial interface with 6dof robot arm
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <DynamixelShield.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>


#define DOF 6
#define OFFSET 90 // Offset angle from sim to real robot
#define UPPER_REAL_GRIP 180
#define LOWER_REAL_GRIP 90
#define UPPER_REAL_ARM 180
#define LOWER_REAL_ARM 0

#define UPPER_SIM_GRIP -45
#define LOWER_SIM_GRIP 0
#define UPPER_SIM_ARM 90
#define LOWER_SIM_ARM -90

using namespace ControlTableItem;

const uint8_t J1 = 1;
const uint8_t J2 = 2;
const uint8_t J3 = 3;
const uint8_t J4 = 4;
const uint8_t J5 = 5;
const uint8_t J6 = 6;
//const uint8_t J7 = 7;

const float DXL_PROTOCOL_VERSION = 1.0;

DynamixelShield dxl;

float gripper;
float arm[DOF];
ros::NodeHandle  nh;

char print_str[15]; // used to print out 15 char outputs

inline float rad_to_deg(float position_radians)
{
  return position_radians * 57.2958;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int val_arm_joints() {
  for(int i = 0; i < DOF; i++){
    if(arm[i] < LOWER_REAL_ARM || arm[i] > UPPER_REAL_ARM){
      return 0;
    }
  }
  return 1;
}

void print_arm(){
  for(int i = 0; i < DOF; i++) {
    dtostrf(arm[i], 9, 3, print_str); // Leave room for too large numbers!
    nh.loginfo(print_str);
  }
}

void drive_arm(){
  if(val_arm_joints()){
		dxl.setGoalPosition(J1, arm[0], UNIT_DEGREE);
		dxl.setGoalPosition(J2, arm[1], UNIT_DEGREE);
		dxl.setGoalPosition(J3, arm[2], UNIT_DEGREE);
		dxl.setGoalPosition(J4, arm[3], UNIT_DEGREE);
		dxl.setGoalPosition(J5, arm[4], UNIT_DEGREE);
		dxl.setGoalPosition(J6, arm[5], UNIT_DEGREE);
  }
  else{
    nh.loginfo("Invalid Arm Joint Value!");
  }
}

/* Callback for JointState subscriber, runs when new JointState
*  is published to move_group/fake_controller_joint_states
*/
void joint_control_cb(const sensor_msgs::JointState& joint_state) { // output can be either for gripper joint or for arm joints
    set_arm_joints(joint_state);
    nh.loginfo("arm");
    print_arm();
    //drive_arm();    
}

/* Sets arm joint values from JointState msg 
*  and maps them to real arm space
*/
void set_arm_joints(const sensor_msgs::JointState& joint_state) {
  arm[0] = rad_to_deg(joint_state.position[0]);
  arm[1] = rad_to_deg(joint_state.position[1]);
  arm[2] = rad_to_deg(joint_state.position[2]);
  arm[3] = rad_to_deg(joint_state.position[3]);
  arm[4] = rad_to_deg(joint_state.position[4]);
  arm[5] = rad_to_deg(joint_state.position[5]);
}

void print_arr(float arr[]){
  for(int i = 0; i < DOF; i++) {
    dtostrf(arr[i], 9, 3, print_str);
    nh.loginfo(print_str);
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("move_group/fake_controller_joint_states", joint_control_cb);

void setup(){
	dxl.begin(1000000);
	dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
	dxl.ping(J1);
	dxl.ping(J2);
	dxl.ping(J3);
	dxl.ping(J4);
	dxl.ping(J5);
	dxl.ping(J6);
	//dxl.ping(J7);

	for(uint8_t i = 1; i <= DOF; i++){ // TODO: add one when adding gripper
		dxl.torqueOff(i);
		dxl.setOperatingMode(i, OP_POSITION);
		dxl.torqueOn(i);
	}

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(5);
}
