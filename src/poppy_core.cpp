#include <signal.h>

#include "DynamixelHandler.h"

#include <ros/ros.h>
#include "advrobotics_lab3/joints.h"
#include "advrobotics_lab3/gripper.h"

// Global variables
float _fps = 10.0f; // Hz
ros::Publisher _jointPositionPublisher;
ros::Publisher _gripperPositionPublisher;
DynamixelHandler _oDxlHandler;
std::string _poppyDxlPortName = "/dev/ttyUSB0";
float _poppyDxlProtocol = 2.0;
int _poppyDxlBaudRate = 1000000;
int _nbJoints = 4;
float _minJointCmd = 0;
float _maxJointCmd = 1023;
float _minJointAngle = -150.0f;
float _maxJointAngle = 150.0f;

float _currentJointAngle_q1 = 0.0f;
float _currentJointAngle_q2 = 0.0f;
float _currentJointAngle_q3 = 0.0f;
float _currentGripperAngle = 0.0f;
 

int convertAnglesToJointCmd(float fJointAngle)
{
	// y = ax + b
	float a =  (_maxJointCmd-_minJointCmd) / (_maxJointAngle - _minJointAngle);
	float b = _minJointCmd - a * _minJointAngle;
	float jointCmd = a * fJointAngle + b;
	return (int)jointCmd;
}

float convertJointCmdToAngles(float fJointCmd)
{
	// y = ax + b
	float a =  (_maxJointAngle - _minJointAngle) / (_maxJointCmd-_minJointCmd);
	float b = _minJointAngle - a * _minJointCmd;
	float jointAngle = a * fJointCmd + b;
	return jointAngle;
}

void goToHomePosition()
{
	std::vector<uint16_t> l_vTargetJointPosition;
	for (int l_joint = 0; l_joint < _nbJoints; l_joint++)
		l_vTargetJointPosition.push_back(convertAnglesToJointCmd(0.0f));
	
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}

void customSigIntHandler(int sig)
{
	ROS_INFO("===Stopping Poppy node===");
	
	// shutdown ROS
	ros::shutdown();
}


void jointCmdCallback(const advrobotics_lab3::joints::ConstPtr& jointAngles)
{
	std::vector<uint16_t> l_vTargetJointPosition;

	//std::cout << "(q1,q2,q3) = (" << jointAngles->q1 << ", " << jointAngles->q2 << ", " << jointAngles->q3 << ")" << std::endl;
	//std::cout << "_currentGripperAngle = (" << _currentGripperAngle << ")" << std::endl;
	
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointAngles->q1));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointAngles->q2));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointAngles->q3));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(_currentGripperAngle));
	
	_currentJointAngle_q1 = jointAngles->q1;
	_currentJointAngle_q2 = jointAngles->q2;
	_currentJointAngle_q3 = jointAngles->q3;
	
	//std::cout << "l_vTargetJointPosition = (" << l_vTargetJointPosition[0] << ", " << l_vTargetJointPosition[1] << ", " <<l_vTargetJointPosition[2] << ", " << l_vTargetJointPosition[3] << ")" << std::endl;
	
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}

void gripperCmdCallback(const advrobotics_lab3::gripper::ConstPtr& gripperAngle)
{
	std::vector<uint16_t> l_vTargetJointPosition;

	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(_currentJointAngle_q1));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(_currentJointAngle_q2));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(_currentJointAngle_q3));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(gripperAngle->gripper));
	
	_currentGripperAngle = gripperAngle->gripper;
	
	//std::cout << "l_vTargetJointPosition= " << _currentJointAngle_q1 << ", " << _currentJointAngle_q2 << ", " << _currentJointAngle_q3 << ", " << gripperAngle->gripper << std::endl;
	
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}


int main(int argc, char** argv)
{
	// create a node called poppy_core
	ros::init(argc, argv, "poppy_core", ros::init_options::NoSigintHandler);
		
	// create a node handle
	ros::NodeHandle nh;
	
	// override the default sigint handler (must be set after the first node handler is created)
	signal(SIGINT, customSigIntHandler);
	
	// create a publisher to joint_position topic
	_jointPositionPublisher = nh.advertise<advrobotics_lab3::joints>("joint_position", 1);
	
	// create a publisher to joint_position topic
	_gripperPositionPublisher = nh.advertise<advrobotics_lab3::gripper>("gripper_position", 1);
	
	// create a subscriber to joint_cmd topic
	ros::Subscriber jointCmdSubscriber = nh.subscribe("joint_cmd", 1, jointCmdCallback);
	
	// create a subscriber to joint_cmd topic
	ros::Subscriber gripperCmdSubscriber = nh.subscribe("gripper_cmd", 1, gripperCmdCallback);
	
	// create a loop rate
	ros::Rate loopRate(_fps);
	
	// create a custom joints message
	advrobotics_lab3::joints jointPositionMsg;
	
	// create a custom gripper message
	advrobotics_lab3::gripper gripperPositionMsg;
		
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.setDeviceName(_poppyDxlPortName);
	_oDxlHandler.setProtocolVersion(_poppyDxlProtocol);
	_oDxlHandler.openPort();
	_oDxlHandler.setBaudRate(_poppyDxlBaudRate);
	_oDxlHandler.enableTorque(true);
	std::cout << std::endl;
	
	goToHomePosition();
	
	ROS_INFO("===Launching Poppy node===");
	
	// loop until Ctrl+C is pressed or ROS connectivity issues
	while(ros::ok())
	{
		//===RETRIEVE Dynamixel Motor positions====
		std::vector<uint16_t> l_vCurrentJointPosition;
		bool bIsReadSuccessfull = _oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
		
		// stores them into a msg
		if (bIsReadSuccessfull && l_vCurrentJointPosition.size() >= 4)
		{
			jointPositionMsg.q1 = convertJointCmdToAngles(l_vCurrentJointPosition[0]);
			jointPositionMsg.q2 = convertJointCmdToAngles(l_vCurrentJointPosition[1]);
			jointPositionMsg.q3 = convertJointCmdToAngles(l_vCurrentJointPosition[2]);
			gripperPositionMsg.gripper = convertJointCmdToAngles(l_vCurrentJointPosition[3]);
			
			/*_currentJointAngle_q1 = convertJointCmdToAngles(l_vCurrentJointPosition[0]);
			_currentJointAngle_q2 = convertJointCmdToAngles(l_vCurrentJointPosition[1]);
			_currentJointAngle_q3 = convertJointCmdToAngles(l_vCurrentJointPosition[2]);
			_currentGripperAngle = convertJointCmdToAngles(l_vCurrentJointPosition[3]);*/
		}
	
		// publish the joints message to the joint_position topic
		_jointPositionPublisher.publish(jointPositionMsg);
		
		// publish the joints message to the joint_position topic
		_gripperPositionPublisher.publish(gripperPositionMsg);
		
		// spin once to let the process handle callback ad key stroke
		ros::spinOnce();
		
		// sleep the right amout of time to comply with _fps 
		loopRate.sleep();
	}
	
	_oDxlHandler.enableTorque(false);
	_oDxlHandler.closePort();
	
	return 0;
}
