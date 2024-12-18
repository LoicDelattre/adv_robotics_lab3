#include "Kinematics.h"


float deg2rad(float angle)
{
	return angle/180.0*M_PI;
}

float rad2deg(float angle)
{
	return angle*180.0/M_PI;
}

std::vector<float> computeForwardKinematics(float q1, float q2, float q3, float L1, float L2, float L3)
{
	float z = L2 * cos(q2) + L3 * cos(q2+q3) + L1;
	float xp = L2 * sin(q2) + L3 * sin(q2+q3);
	float x = xp * cos(q1);
	float y = xp * sin(q1);
	//std::cout << "[INFO] Forward Kinematics : (q1, q2, q3)->(x, y, z) = (" << rad2deg(q1) << ", " << rad2deg(q2) << ", " << rad2deg(q3) << ")->(" << x << ", " << y << ", " << z << ")" << std::endl;
	std::vector<float> X;
	X.push_back(x);
	X.push_back(y);
	X.push_back(z);
	
	return X;
}

std::vector<float> computeInverseKinematics(float x, float y, float z, float L1, float L2, float L3)
{
	std::vector<float> qi;
	
	// Determines q1
	float q1 = atan2(y, x);
	
	// Determines cos(q3) to find out the number of solutions
	float xp = sqrt(x*x + y*y);
	float zc = z - L1;
	float cos_q3 = (zc*zc+xp*xp-(L2*L2+L3*L3)) / (2.0 * L2 * L3);
	
	//std::cout << "[INFO] cos_q3= " << cos_q3 << std::endl;
	
	if (cos_q3 >1 | cos_q3 <-1)
	{
		qi.push_back(0.0);
		std::cout << "[INFO] Inverse Kinematics: No solution!" << std::endl;
	} 
	else if (cos_q3 == 1)
	{
		qi.push_back(1.0);
		float q2 = atan2(xp, zc);
		float q3 = 0;
		std::cout << "[INFO] Inverse Kinematics: One solution: (x, y, z)->(q1, q2, q3) = (" << x << ", " << y << ", " << z << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ", " << rad2deg(q3) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
		qi.push_back(q3);
	}
	else if (cos_q3 == -1)
	{
		qi.push_back(1.0);
		float q2 = atan2(xp, zc);
		float q3 = M_PI;
		std::cout << "[INFO] Inverse Kinematics: One solution: (x, y, z)->(q1, q2, q3) = (" << x << ", " << y << ", " << z << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ", " << rad2deg(q3) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
		qi.push_back(q3);
	}
	else
	{
		qi.push_back(2.0);
		std::cout << "[INFO] Inverse Kinematics: Two solutions: "<< std::endl;
		
		float q3 = acos(cos_q3);
		float q2 = (float)(atan2(xp, zc) - atan2(L2*sin(q3), L1+L2*cos_q3));
		std::cout << "\t(x, y)->(q1, q2, q3) = (" << x << ", " << y  << ", " << z << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ", " << rad2deg(q3) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
		qi.push_back(q3);
		
		q3 = -acos(cos_q3);
		q2 = (float)(atan2(xp, zc) - atan2(L2*sin(q3), L1+L2*cos_q3));
		
		std::cout << "\t(x, y)->(q1, q2, q3) = (" << x << ", " << y  << ", " << z << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ", " << rad2deg(q3) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
		qi.push_back(q3);
	}
	
	return qi;
}

/*std::vector<float> computeDifferentialKinematics(float q1, float q2, float q3, float L1, float L2, float L3)
{
	std::vector<float> jacobian;
	
	float j11 = -sin(q1)*(L2*sin(q2) + L3*sin(q2+q3));
	float j12 = L2*cos(q1)*cos(q2)+L3*cos(q1)*cos(q2+q3);
	float j13 = L3*cos(q1)*cos(q2+q3);
	
	float j21 = cos(q1)*(L2*sin(q2)+L3*sin(q2+q3)); 
	float j22 = L2*sin(q1)*cos(q2)+L3*sin(q1)*cos(q2+q3);
	float j23 = L3*sin(q1)*cos(q2+q3);
	
	float j31 = 0; 
	float j32 = -L2*sin(q2)-L3*sin(q2+q3);
	float j33 = -L3*sin(q2+q3);
	
	jacobian.push_back(j11); jacobian.push_back(j12); jacobian.push_back(j13);
	jacobian.push_back(j21); jacobian.push_back(j22); jacobian.push_back(j23);
	jacobian.push_back(j31); jacobian.push_back(j32); jacobian.push_back(j33);
	
	return jacobian;
}

int computeJacobianMatrixRank(std::vector<float> vJacobianMatrix, float threshold)
{
	int rank = -1;
	cv::Mat1f oJacobianMatrix(3, 3);
	
	if (vJacobianMatrix.size() == 9)
	{
		// Converts the Jacobian matrix from std::vector to cv::Mat
		oJacobianMatrix.at<float>(0, 0) = vJacobianMatrix[0];
		oJacobianMatrix.at<float>(0, 1) = vJacobianMatrix[1];
		oJacobianMatrix.at<float>(0, 2) = vJacobianMatrix[2];
		oJacobianMatrix.at<float>(1, 0) = vJacobianMatrix[3];
		oJacobianMatrix.at<float>(1, 1) = vJacobianMatrix[4];
		oJacobianMatrix.at<float>(1, 2) = vJacobianMatrix[5];
		oJacobianMatrix.at<float>(2, 0) = vJacobianMatrix[6];
		oJacobianMatrix.at<float>(2, 1) = vJacobianMatrix[7];
		oJacobianMatrix.at<float>(2, 2) = vJacobianMatrix[8];
		std::cout << "=====Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(0,0) << ", " <<  oJacobianMatrix.at<float>(0,1)  << ", " <<  oJacobianMatrix.at<float>(0,2) << " ]" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(1,0) << ", " <<  oJacobianMatrix.at<float>(1,1)  << ", " <<  oJacobianMatrix.at<float>(1,2)<< " ]" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(2,0) << ", " <<  oJacobianMatrix.at<float>(2,1)  << ", " <<  oJacobianMatrix.at<float>(2,2)<< " ]" << std::endl;
		// Computes the determinant of the Jacobian matrix
		float determinant = abs(-vJacobianMatrix[7]*(vJacobianMatrix[0] * vJacobianMatrix[5] - vJacobianMatrix[3]*vJacobianMatrix[2])+vJacobianMatrix[8]*(vJacobianMatrix[0] * vJacobianMatrix[4] - vJacobianMatrix[3]*vJacobianMatrix[3]));
		std::cout << "=====Determinant of the Jacobian matrix=====" << std::endl << determinant << std::endl;
		// Computes SVD
		cv::Mat1f w, u, vt;
		cv::SVD::compute(oJacobianMatrix, w, u, vt);
		// Finds non zero singular values
		cv::Mat1f nonZeroSingularValues = w/w.at<float>(0,0) > threshold;
		// Counts the number of non zero singular values
		rank = cv::countNonZero(nonZeroSingularValues);
		std::cout << "=====Rank of the Jacobian matrix=====" << std::endl << rank << " / " << oJacobianMatrix.rows << std::endl;
		// Determines the inverse of the Jacobian matrix
		cv::Mat oJacobianInverse =  oJacobianMatrix.inv(); 
		std::cout << "=====Inverse of the Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(0,0) << ", " <<  oJacobianInverse.at<float>(0,1)<< ", " <<  oJacobianInverse.at<float>(0,2)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(1,0) << ", " <<  oJacobianInverse.at<float>(1,1)  << ", " <<  oJacobianInverse.at<float>(1,2)<< " ]" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(2,0) << ", " <<  oJacobianInverse.at<float>(2,1)  << ", " <<  oJacobianInverse.at<float>(2,2)<< " ]" << std::endl;
	}
	else
		std::cout << "[ERROR] Jacobian matrix has a size of "<< vJacobianMatrix.size() << " instead of 4" << std::endl;

	return rank;
}

cv::Mat  computeInverseJacobianMatrix(std::vector<float> vJacobianMatrix)
{
	cv::Mat1f oJacobianMatrix(2, 2);
	cv::Mat oJacobianInverse;
	
	if (vJacobianMatrix.size() == 4)
	{
		// Converts the Jacobian matrix from std::vector to cv::Mat
		oJacobianMatrix.at<float>(0, 0) = vJacobianMatrix[0];
		oJacobianMatrix.at<float>(0, 1) = vJacobianMatrix[1];
		oJacobianMatrix.at<float>(1, 0) = vJacobianMatrix[2];
		oJacobianMatrix.at<float>(1, 1) = vJacobianMatrix[3];
		std::cout << "=====Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(0,0) << ", " <<  oJacobianMatrix.at<float>(0,1)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(1,0) << ", " <<  oJacobianMatrix.at<float>(1,1)  << " ]" << std::endl;
		// Determines the inverse of the Jacobian matrix
		cv::invert(oJacobianMatrix, oJacobianInverse, cv::DECOMP_SVD);
		//oJacobianInverse =  oJacobianMatrix.inv(); 
		std::cout << "=====Inverse of the Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(0,0) << ", " <<  oJacobianInverse.at<float>(0,1)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(1,0) << ", " <<  oJacobianInverse.at<float>(1,1)  << " ]" << std::endl;
	}
	else
		std::cout << "[ERROR] Jacobian matrix has a size of "<< vJacobianMatrix.size() << " instead of 4" << std::endl;

	return oJacobianInverse;
}*/
