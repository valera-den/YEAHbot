#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
/*
Joint 0: 0.0100692 - 5.84014
Joint 1: 0.0100692 - 2.61799
Joint 2: -5.02655  - -0.215708
Joint 3: 0.0221239 - 3.4292
Joint 4: 0.110619  - 5.64159
*/

ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) 
{
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) 
	{
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) 
{
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}

using namespace std;
static struct termios stored_settings;
     
void moveArm_Home() 
{
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	// move arm back to home position
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.111;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(5).sleep();
}

void moveArm_Up() 
{
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	// move arm back to home position
	jointvalues[0] = 2.95;
	jointvalues[1] = 1.05;
	jointvalues[2] = -2.44;
	jointvalues[3] = 1.73;
	jointvalues[4] = 2.95;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(3).sleep();
}

void set_keypress(void)
{
	struct termios new_settings;

	tcgetattr(0,&stored_settings);

	new_settings = stored_settings;

	/* Disable canonical mode, and set buffer size to 1 byte */
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	new_settings.c_cc[VMIN] = 1;

	tcsetattr(0,TCSANOW,&new_settings);
	return;
}

void reset_keypress(void)
{
	tcsetattr(0,TCSANOW,&stored_settings);
	return;
}

int main(int argc, char **argv)
{
	set_keypress();
	char Pressed_Key = 0;
	bool Flag_Move_Arm_Joints = false;
	double Current_Arm_Joint_Position = 0.11;
	int Current_Arm_Joint = 0;

	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

	ROS_INFO("getch test");
	while (ros::ok())
	{
		cin >> Pressed_Key;
		cout << "pressed: " << Pressed_Key << endl;

		if(Flag_Move_Arm_Joints == false)
		{
			switch(Pressed_Key)
			{
				case 'h':
					moveArm_Home();
				break;

				case 'u':
					moveArm_Up();
				break;

				case 'j':
					Flag_Move_Arm_Joints = true;
					ROS_INFO("Move arm by joints");
				break;
			}
		}
		else if(Flag_Move_Arm_Joints == true)
		{
			//ROS_INFO("Move arm by joints");
			brics_actuator::JointPositions msg;
			std::vector<double> jointvalues(5);

			switch(Pressed_Key)
			{
				case 'j':
					Flag_Move_Arm_Joints = false;
					ROS_INFO("Move arm by positions");
				break;

				case '0':
					Current_Arm_Joint = 0;
					cout << "Current_Arm_Joint: " << Current_Arm_Joint << endl;
				break;

				case '1':
					Current_Arm_Joint = 1;
					cout << "Current_Arm_Joint_Position: " << Current_Arm_Joint << endl;
				break;

				case ']':
					Current_Arm_Joint_Position = Current_Arm_Joint_Position + 0.1;
					jointvalues[0] = Current_Arm_Joint_Position;	
					cout << "Current_Arm_Joint_Position: " << Current_Arm_Joint_Position << endl;				
				break;

				case '[':
					Current_Arm_Joint_Position = Current_Arm_Joint_Position - 0.1;
					jointvalues[Current_Arm_Joint] = Current_Arm_Joint_Position - 0.1;
					cout << "Current_Arm_Joint_Position: " << Current_Arm_Joint_Position << endl;
				break;
			}	
			jointvalues[1] = 0.11;
			jointvalues[2] = -0.11;
			jointvalues[3] = 0.11;
			jointvalues[4] = 0.111;
			msg = createArmPositionCommand(jointvalues);
			armPublisher.publish(msg);
			ros::Duration(0.5).sleep();

		}
		
	}

	sleep(1);
	ros::shutdown();
	return 0;
}
