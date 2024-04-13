#include "KinChain.hpp"

Manipulator_2D2J::Manipulator_2D2J(){
	// define robot inital state
	linkLenght_[0] = 150;
	linkLenght_[1] = 150;
	linkLenght_[2] = 100;
	jValue_[0] = 0; // phi0
	jValue_[1] = 0; // phi1

	buildRobot();
	return;
};

Manipulator_2D2J::~Manipulator_2D2J(){
	if(this->T_1_0_ != NULL) delete this->T_1_0_;
	if(this->T_2_1_ != NULL) delete this->T_2_1_;
	if(this->T_3_2_ != NULL) delete this->T_3_2_;
	return;
};

int Manipulator_2D2J::getNmbJoints(){
	return nmbJoints_;
}

void Manipulator_2D2J::buildRobot(){
	// define matrices
	T_1_0_ = factory('Z',-jValue_[0],0  ,linkLenght_[0],0);
	T_2_1_ = factory('Z',-jValue_[1],linkLenght_[1],0  ,0);
	T_3_2_ = factory('Z', 0    ,linkLenght_[2],0  ,0);

	// calculate final matrices
	cv::Mat T_2_0 = T_1_0_->current() * T_2_1_->current();
	cv::Mat T_3_0 = T_2_0  * T_3_2_->current();

	// read joint coordinates (incl. TCP)
	// base
	jCoord_[0] = cv::Point3d(0,0,0);

	// joint 0
	jCoord_[1] = cv::Point3d(T_1_0_->current().at<float>(0,3),
							 T_1_0_->current().at<float>(1,3),
							 T_1_0_->current().at<float>(2,3));

	// joint 1
	jCoord_[2] = cv::Point3d(T_2_0.at<float>(0,3),
							 T_2_0.at<float>(1,3),
							 T_2_0.at<float>(2,3));

	// TCP
	jCoord_[3] = cv::Point3d(T_3_0.at<float>(0,3),
							 T_3_0.at<float>(1,3),
							 T_3_0.at<float>(2,3));

	return;
}



void Manipulator_2D2J::getRobotCoord(cv::Point3d points[]){
	for(int i=0; i < nmbRobotPoints_; i++){
		points[i] = jCoord_[i];
	}
	return;
}

void Manipulator_2D2J::getJointValues(float v[]){
	for(int i=0; i < nmbJoints_; i++){
		v[i] = jValue_[i];
	}
	return;
}


void Manipulator_2D2J::setJointValues(const float v[]){
	for(int i=0; i < nmbJoints_; i++){
		jValue_[i] = v[i];
	}
	this->buildRobot();
	return;
}


int  Manipulator_2D2J::getNmbRobotPoints(){
	return nmbRobotPoints_;
}

void Manipulator_2D2J::setTCPCoord(cv::Point3d){
	throw std::string("nyi");
}


