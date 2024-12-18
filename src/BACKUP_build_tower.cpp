#include "math.h"

#include "ros/ros.h"
#include "advrobotics_lab3/ikpoppy.h"
#include "advrobotics_lab3/joints.h"
#include "advrobotics_lab3/gripper.h"

#include "Kinematics.h"

float _fps = 5.0;
int nbRepetitionsCmd = 20;
double deg2rad(double angle)
{
	return -angle / 180.0 * M_PI;
}


int main(int argc, char** argv)
{
	// inits the node 
	ros::init(argc, argv, "build_tower");
	ros::NodeHandle nh;
	ros::Rate loopRate(_fps);
	int counter = 0;
	
	// checks the number of args
	if (argc != 7)
	{
		ROS_INFO("usage: rosrun advrobotics_lab3 build_tower x0 y0 z0 x1 y1 z1");
		ROS_INFO("ex.: rosrun advrobotics_lab3 ik_client 6 9 0 6 0 0");
		return 1;
	}
		
	// inits the ik service
	ros::ServiceClient client = nh.serviceClient<advrobotics_lab3::ikpoppy>("ik");
	advrobotics_lab3::ikpoppy srv;
	// initis the topics
	ros::Publisher jointCmdPublisher = nh.advertise<advrobotics_lab3::joints>("joint_cmd", 1);
	advrobotics_lab3::joints  jointCmdMsg;
	ros::Publisher gripperCmdPublisher = nh.advertise<advrobotics_lab3::gripper>("gripper_cmd", 1);
	advrobotics_lab3::gripper  gripperCmdMsg;
	
	// 0- goes to home position
	ROS_INFO("==> 0- goes to home position");
	jointCmdMsg.q1 = 0.0;
	jointCmdMsg.q2 = 0.0;
	jointCmdMsg.q3 = 0.0;
	while (ros::ok && counter < nbRepetitionsCmd)
	{
		jointCmdPublisher.publish(jointCmdMsg);
		ros::spinOnce();
		loopRate.sleep();
		counter++;
	}
		
	// 1- opens the gripper
	ROS_INFO("==> 1- opens the gripper");
	counter = 0;
	gripperCmdMsg.gripper = -40.0;
	while (ros::ok && counter < nbRepetitionsCmd)
	{
		gripperCmdPublisher.publish(gripperCmdMsg);
		ros::spinOnce();
		loopRate.sleep();
		counter++;
	}
	
	// 2- moves to the intial position
	ROS_INFO("==> 2- moves to the intial position");
	counter = 0;
	srv.request.x = atof(argv[1]);
	srv.request.y = atof(argv[2]);
	srv.request.z = atof(argv[3]);
	
	if (client.call(srv))
	{
		ROS_INFO("IK: (x,y,z) --> (J1, J2, j3)");
		ROS_INFO("(%f, %f, %f) --> (%f, %f, %f)", atof(argv[1]), atof(argv[2]), atof(argv[3]), rad2deg(srv.response.q1), rad2deg(srv.response.q2), rad2deg(srv.response.q3));
		
		jointCmdMsg.q1 = rad2deg(srv.response.q1);
		jointCmdMsg.q2 = rad2deg(srv.response.q2);
		jointCmdMsg.q3 = rad2deg(srv.response.q3);
		
		while (ros::ok && counter < nbRepetitionsCmd)
		{
			jointCmdPublisher.publish(jointCmdMsg);
			ros::spinOnce();
			loopRate.sleep();
			counter++;
		}
	}
	else
	{
		ROS_ERROR("Failed to call the service ik!");
		return 1;
	}
	
	// 3- closes the gripper
	ROS_INFO("==> 3- closes the gripper");
	counter = 0;
	gripperCmdMsg.gripper = 10.0;
	while (ros::ok && counter < nbRepetitionsCmd)
	{
		gripperCmdPublisher.publish(gripperCmdMsg);
		ros::spinOnce();
		loopRate.sleep();
		counter++;
	}
	
	// 4- moves to the final position
	ROS_INFO("==> 2- moves to the intial position");
	counter = 0;
	srv.request.x = atof(argv[4]);
	srv.request.y = atof(argv[5]);
	srv.request.z = atof(argv[6]);
	
	if (client.call(srv))
	{
		ROS_INFO("IK: (x,y,z) --> (J1, J2, j3)");
		ROS_INFO("(%f, %f, %f) --> (%f, %f, %f)", atof(argv[1]), atof(argv[2]), atof(argv[3]), rad2deg(srv.response.q1), rad2deg(srv.response.q2), rad2deg(srv.response.q3));
		
		jointCmdMsg.q1 = rad2deg(srv.response.q1);
		jointCmdMsg.q2 = rad2deg(srv.response.q2);
		jointCmdMsg.q3 = rad2deg(srv.response.q3);
		
		while (ros::ok && counter < nbRepetitionsCmd)
		{		
			jointCmdPublisher.publish(jointCmdMsg);
			ros::spinOnce();
			loopRate.sleep();
			counter++;
		}
	}
	else
	{
		ROS_ERROR("Failed to call the service ik!");
		return 1;
	}
	
	// 5- opens the gripper
	ROS_INFO("==> 5- opens the gripper");
	counter = 0;
	gripperCmdMsg.gripper = -40.0;
	while (ros::ok && counter < nbRepetitionsCmd)
	{
		gripperCmdPublisher.publish(gripperCmdMsg);
		ros::spinOnce();
		loopRate.sleep();
		counter++;
	}
	
	// 6-moves to home position
	ROS_INFO("==> 6-moves to home position");
	counter=0;
	jointCmdMsg.q1 = 0.0;
	jointCmdMsg.q2 = 0.0;
	jointCmdMsg.q3 = 0.0;
	
	while (ros::ok && counter < nbRepetitionsCmd)
	{
		jointCmdPublisher.publish(jointCmdMsg);
		ros::spinOnce();
		loopRate.sleep();
		counter++;
	}
		
	return 0;
}