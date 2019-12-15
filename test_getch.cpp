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
//test git 2
/*
Joint 0: 0.0100692 - 5.84014		0.6 - 285.5 grad
Joint 1: 0.0100692 - 2.61799		0.6 - 149.9 grad
Joint 2: -5.02655  - -0.215708		-288 - -12.36 grad
Joint 3: 0.0221239 - 3.4292			1.27 - 196.47 grad
Joint 4: 0.110619  - 5.64159		6.34 - 323.24 grad
*/
double Array_Joints_Range[5][2] =
{
{0.0100692, 5.84014},
{0.0100692, 2.61799},
{-5.02655, -0.215708},
{0.0221239, 3.4292},
{0.110619, 5.64159}
};

char Pressed_Key = 0;
bool Flag_Move_Arm_Joints = false;
double Current_Arm_Joint_Position = 0.11;
int Current_Arm_Joint = 0;
double Step_Position = 0.1;

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

brics_actuator::JointPositions msg;
std::vector<double> jointvalues(5);

using namespace std;
static struct termios stored_settings;

void moveArm_Home()
{
	//brics_actuator::JointPositions msg;
	//std::vector<double> jointvalues(5);

	// move arm back to home position
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.111;

	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(2).sleep();
}

void moveArm_Up()
{
	//brics_actuator::JointPositions msg;
	//std::vector<double> jointvalues(5);

	// move arm back to home position
	jointvalues[0] = 2.95;
	jointvalues[1] = 1.05;
	jointvalues[2] = -2.44;
	jointvalues[3] = 1.73;
	jointvalues[4] = 2.95;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(2).sleep();
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

void Set_Joint_Position(int Number_Of_Joint, double Angle)
{
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.111;
	jointvalues[Number_Of_Joint] = Angle;
}

void Check_Joint_Range(int Joint_Number)
{
	if(jointvalues[Joint_Number] < Array_Joints_Range[Joint_Number][0])
	{
		jointvalues[Joint_Number] = Array_Joints_Range[Joint_Number][0];
		Current_Arm_Joint_Position = Array_Joints_Range[Joint_Number][0];
		ROS_INFO("Joint is out of minimum range");
	}
	else if(jointvalues[Joint_Number] > Array_Joints_Range[Joint_Number][1])
	{
		jointvalues[Joint_Number] = Array_Joints_Range[Joint_Number][1];
		Current_Arm_Joint_Position = Array_Joints_Range[Joint_Number][1];
		ROS_INFO("Joint is out of maximum range");
	}
}

void Arm_Publish()
{
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
	ros::Duration(0.5).sleep();
}

int main(int argc, char **argv)
{
	set_keypress();

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

		if(Pressed_Key == '`')
		{
			ROS_INFO("Exit");
			ros::shutdown();
		}

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
			//brics_actuator::JointPositions msg;
			//std::vector<double> jointvalues(5);

			switch(Pressed_Key)
			{
				case 'j':
					Flag_Move_Arm_Joints = false;
					ROS_INFO("Move arm by positions");
				break;

				case 'q':
					ROS_INFO("Big step position");
					Step_Position = 0.4;
				break;

				case 'w':
					ROS_INFO("Middle step position");
					Step_Position = 0.15;
				break;

				case 'e':
					ROS_INFO("Low arm by positions");
					Step_Position = 0.02;
				break;

				case '0':
					Current_Arm_Joint = 0;
					Current_Arm_Joint_Position = jointvalues[0];
					cout << "Current_Arm_Joint: " << Current_Arm_Joint << endl;
				break;

				case '1':
					Current_Arm_Joint = 1;
					Current_Arm_Joint_Position = jointvalues[1];
					cout << "Current_Arm_Joint: " << Current_Arm_Joint << endl;
				break;

				case '2':
					Current_Arm_Joint = 2;
					Current_Arm_Joint_Position = jointvalues[2];
					cout << "Current_Arm_Joint: " << Current_Arm_Joint << endl;
				break;

				case '3':
					Current_Arm_Joint = 3;
					Current_Arm_Joint_Position = jointvalues[3];
					cout << "Current_Arm_Joint: " << Current_Arm_Joint << endl;
				break;

				case '4':
					Current_Arm_Joint = 4;
					Current_Arm_Joint_Position = jointvalues[4];
					cout << "Current_Arm_Joint: " << Current_Arm_Joint << endl;
				break;

				case ']':
					Current_Arm_Joint_Position = Current_Arm_Joint_Position + Step_Position;
					jointvalues[Current_Arm_Joint] = Current_Arm_Joint_Position;
					Check_Joint_Range(Current_Arm_Joint);
					cout << "Current_Arm_Joint_Position: " << Current_Arm_Joint_Position << endl;
					Arm_Publish();
				break;

				case '[':
					Current_Arm_Joint_Position = Current_Arm_Joint_Position - Step_Position;
					jointvalues[Current_Arm_Joint] = Current_Arm_Joint_Position;
					Check_Joint_Range(Current_Arm_Joint);
					cout << "Current_Arm_Joint_Position: " << Current_Arm_Joint_Position << endl;
					Arm_Publish();
				break;
			}
		}

	}

	sleep(1);
	ros::shutdown();
	return 0;
}
