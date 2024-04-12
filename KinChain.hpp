/**
 *
 * \file KinChain.hpp
 *
 * \brief Contains class declarations for...
 *
 *
 *
 */
#ifndef KINCHAIN_HPP_
#define KINCHAIN_HPP_


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "HTM.hpp"


class IKinChain{
public:

	virtual int  getNmbJoints()                = 0;
	virtual int  getNmbRobotPoints()           = 0;
	virtual void getRobotCoord(cv::Point3d[])  = 0;
	virtual void getJointValues(float[])       = 0;
	virtual void setJointValues(const float[]) = 0;
	virtual void setTCPCoord(cv::Point3d)      = 0;
	virtual void buildRobot()                  = 0;
};



class Manipulator_2D2J : public IKinChain{
public:
	Manipulator_2D2J();
	~Manipulator_2D2J();
	int  getNmbJoints();
	int  getNmbRobotPoints();
	void getRobotCoord(cv::Point3d[]);
	void getJointValues(float[]);
	void setJointValues(const float[]);
	void setTCPCoord(cv::Point3d);
	void buildRobot();
protected:
	static const int nmbJoints_ = 2;
	static const int nmbLinks_  = 3;
	static const int nmbRobotPoints_ = 4; // position base, joints and TCP

	float linkLenght_[nmbLinks_];  // links
	float jValue_[nmbJoints_]; // rotations joints values
	cv::Point3d jCoord_[nmbRobotPoints_];


	IHTM *T_1_0_ = NULL;
	IHTM *T_2_1_ = NULL;
	IHTM *T_3_2_ = NULL;
};




#endif /* KINCHAIN_HPP_ */
