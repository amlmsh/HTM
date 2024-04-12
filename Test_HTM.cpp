#include "HTM.hpp"
#include "KinChain.hpp"

#include <iostream>

#define PI 3.14159

int main(){

	Manipulator_2D2J robot;
	float joint[2];

	cv::Point3d robotPoints[robot.getNmbRobotPoints()];
	cv::Point3d tcp;

	robot.getRobotCoord(robotPoints);
	tcp = robotPoints[robot.getNmbRobotPoints() - 1];
	std::cout << tcp << std::endl;

	joint[0] = PI/3;
	joint[1] = PI/4;
	robot.setJointValues(joint);
	robot.getRobotCoord(robotPoints);
	tcp = robotPoints[robot.getNmbRobotPoints() - 1];
	std::cout << tcp << std::endl;



    return 0;
}
