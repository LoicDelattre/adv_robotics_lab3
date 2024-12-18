#include "math.h"

#include "ros/ros.h"
#include "advrobotics_lab3/ikpoppy.h"
#include "advrobotics_lab3/joints.h"

#include "Kinematics.h"

float _fps = 10.0;

double deg2rad(double angle)
{
	return -angle / 180.0 * M_PI;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ik_client");
	
	if (argc != 4)
	{
		ROS_INFO("usage: rosrun advrobotics_lab3 ik_client x y z");
		ROS_INFO("ex.: rosrun advrobotics_lab3 ik_client 6 9 0");
		return 1;
	}
	
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<advrobotics_lab3::ikpoppy>("ik");
	
	ros::Publisher jointCmdPublisher = nh.advertise<advrobotics_lab3::joints>("joint_cmd", 1);
	
	advrobotics_lab3::ikpoppy srv;
	srv.request.x = atof(argv[1]);
	srv.request.y = atof(argv[2]);
	srv.request.z = atof(argv[3]);
	
	// create a loop rate
	ros::Rate loopRate(_fps);
	
	if (client.call(srv))
	{
		ROS_INFO("IK: (x,y,z) --> (J1, J2, j3)");
		ROS_INFO("(%f, %f, %f) --> (%f, %f, %f)", atof(argv[1]), atof(argv[2]), atof(argv[3]), rad2deg(srv.response.q1), rad2deg(srv.response.q2), rad2deg(srv.response.q3));
		
		while (ros::ok())
		{
			advrobotics_lab3::joints  jointCmdMsg;
			jointCmdMsg.q1 = rad2deg(srv.response.q1);
			jointCmdMsg.q2 = rad2deg(srv.response.q2);
			jointCmdMsg.q3 = rad2deg(srv.response.q3);
			
			jointCmdPublisher.publish(jointCmdMsg);
			ros::spinOnce();
			loopRate.sleep();
		}			
			
	}
	else
	{
		ROS_ERROR("Failed to call the service ik!");
		return 1;
	}
	
	return 0;
}