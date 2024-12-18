#include "ros/ros.h"

#include "advrobotics_lab3/ikpoppy.h"

#include "Kinematics.h"

bool ik(advrobotics_lab3::ikpoppy::Request &request, advrobotics_lab3::ikpoppy::Response &response)
{
	ROS_INFO("================================================");
	ROS_INFO("IK for (%f, %f, %f) in progress...", request.x, request.y, request.z);

	std::vector<float> qi = computeInverseKinematics(request.x, request.y, request.z, LINK1, LINK2, LINK3);
	
	response.sol = qi[0];
	response.q1 = qi[1];
	response.q2 = qi[2];
	response.q3 = qi[3];
		
	ROS_INFO("IK: (x,y,z) --> (J1, J2, J3)");
	ROS_INFO("(%f, %f, %f) --> (%f, %f, %f)", request.x, request.y, request.z, rad2deg(response.q1), rad2deg(response.q2), rad2deg(response.q3));
	ROS_INFO("-----------------------------------------------------------------------------------------------------------");
	
	return true;
}


int main(int argc, char** argv)
{
	// args
	if (argc != 1)
	{
		ROS_INFO("usage: rosrun advrobotics_lab3 ik_server");
		return 1;
	}
			
	// ROS
	ros::init(argc, argv, "ik_server");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("ik", ik);
	
	ROS_INFO("IK server launched...");
	
	ros::spin();
	
	return 0;
}