#include "KinChain.hpp"

#include <iostream>

using namespace std;

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


PanTilt::PanTilt(){
	panValueRad_ = 0;
	tiltValueRad_ = 0;
	link0_ = 0;
	link1_ = 0;
	this->buildRobot();
	return;
}

PanTilt::~PanTilt(){
	return;
}

int  PanTilt::getNmbJoints(){
	return 2;
}

int  PanTilt::getNmbRobotPoints(){
	return 3;
}

void PanTilt::getRobotCoord(cv::Point3d p[]){
	this->buildRobot();

	// read joint coordinates (incl. TCP)
	// base (with joint 0)
	p[0] = cv::Point3d(0,0,0);

	// joint 1
	p[1] = cv::Point3d(T_1_0_.current().at<float>(0,3),
							 T_1_0_.current().at<float>(1,3),
							 T_1_0_.current().at<float>(2,3));

	// TCP
	p[2] = cv::Point3d(T_2_0_.at<float>(0,3),
							 T_2_0_.at<float>(1,3),
							 T_2_0_.at<float>(2,3));


	return;
}

void PanTilt::setTCPCoord(cv::Point3d){
	throw std::string("not implemented.");
}

void PanTilt::getJointValues(float j[]){
	j[PAN_IDX_] = panValueRad_;
	j[TILT_IDX_] = tiltValueRad_;
	return;
}

void PanTilt::setJointValues(const float j[]){
	panValueRad_ = j[PAN_IDX_];
	tiltValueRad_ = j[TILT_IDX_];
	this->buildRobot();
	return;
}


void PanTilt::buildRobot(){
	/**
	 *
	 * DH Parameter for the pan-tilt-kinematic
	 *
	 *  n  |     Phi_n      |  Alpha_n |   a_n   |   d_n
	 * ---------------------------------------------------
	 *   1 |  panValueRad_  | + PI/2   |    0    | link0_
	 *   2 |  tiltValuRad_  | - PI/2   | link1_0 |   0
	 *
	 */


	DH_Param_1_[DH_PARAM_PHI_IDX_]   = panValueRad_;
	DH_Param_1_[DH_PARAM_ALPHA_IDX_] = 0.5*3.14159;
	DH_Param_1_[DH_PARAM_A_IDX_]     = 0;
	DH_Param_1_[DH_PARAM_D_IDX_]     = link0_;

	DH_Param_2_[DH_PARAM_PHI_IDX_]   = tiltValueRad_;
	DH_Param_2_[DH_PARAM_ALPHA_IDX_] = -0.5*3.14159;;
	DH_Param_2_[DH_PARAM_A_IDX_]     = link1_;
	DH_Param_2_[DH_PARAM_D_IDX_]     = 0;


	T_1_0_.updateDHparam(DH_Param_1_[DH_PARAM_PHI_IDX_],
			             DH_Param_1_[DH_PARAM_ALPHA_IDX_],
						 DH_Param_1_[DH_PARAM_A_IDX_],
						 DH_Param_1_[DH_PARAM_D_IDX_]);

	T_2_1_.updateDHparam(DH_Param_2_[DH_PARAM_PHI_IDX_],
			             DH_Param_2_[DH_PARAM_ALPHA_IDX_],
						 DH_Param_2_[DH_PARAM_A_IDX_],
						 DH_Param_2_[DH_PARAM_D_IDX_]);

	T_2_0_ = T_1_0_.current() * T_2_1_.current();
	cv::invert(T_2_0_,T_0_2_);
	return;
}


cv::Mat PanTilt::getForwardKin(){
	return T_2_0_;

}

cv::Mat PanTilt::getInvertedForwardKin(){
	return T_0_2_;
}





PanTiltActiveVisionSystem::PanTiltActiveVisionSystem(int width, int height, int FOV){
	panTilt_ = new PanTilt();
	cam_ = new Cam(width, height, FOV);
	return;
}

PanTiltActiveVisionSystem::~PanTiltActiveVisionSystem(){
	delete panTilt_;
	delete cam_;
	return;
}


void    PanTiltActiveVisionSystem::setPan(float angleRad){
	float valuesRad[2];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->PAN_IDX_] = angleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}

void    PanTiltActiveVisionSystem::setTilt(float angleRad){
	float valuesRad[2];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->TILT_IDX_] = angleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}


void    PanTiltActiveVisionSystem::setDeltaPan(float deltaAngleRad){
	float valuesRad[2];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->PAN_IDX_] += deltaAngleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}

void    PanTiltActiveVisionSystem::setDeltaTilt(float deltaAngleRad){
	float valuesRad[2];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->TILT_IDX_] += deltaAngleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}

float   PanTiltActiveVisionSystem::getPan(){
	float valuesRad[2];
	panTilt_->getJointValues(valuesRad);
	return valuesRad[panTilt_->PAN_IDX_];
}

float   PanTiltActiveVisionSystem::getTilt(){
	float valuesRad[2];
	panTilt_->getJointValues(valuesRad);
	return valuesRad[panTilt_->TILT_IDX_];
}

int     PanTiltActiveVisionSystem::getWidth(){
	return cam_->getWidth();
}


int     PanTiltActiveVisionSystem::getHeight(){
	return cam_->getHeight();
}

cv::Mat PanTiltActiveVisionSystem::getImgData(cv::Point3d p){
	cv::Mat invFK = panTilt_->getInvertedForwardKin();
	cv::Point3d pCamCoordSys;

	cv::Mat pHomo = cv::Mat::zeros(4,1,CV_32F);
	cv::Mat pHomoRes = cv::Mat::zeros(4,1,CV_32F);
	pHomo.at<float>(0,0) = p.x;
	pHomo.at<float>(1,0) = p.y;
	pHomo.at<float>(2,0) = p.z;
	pHomo.at<float>(3,0) = 1.0;

	pHomoRes = invFK * pHomo;

	pCamCoordSys.x = pHomoRes.at<float>(0,0);
	pCamCoordSys.y = pHomoRes.at<float>(1,0);
	pCamCoordSys.z = pHomoRes.at<float>(2,0);

	return (cam_->getImgData(pCamCoordSys));
}

void    PanTiltActiveVisionSystem::getPixelData(cv::Point3d p, int pixel[2]){
	cv::Mat invFK = panTilt_->getInvertedForwardKin();
	cv::Point3d pCamCoordSys;

	cv::Mat pHomo = cv::Mat::zeros(4,1,CV_32F);
	cv::Mat pHomoRes = cv::Mat::zeros(4,1,CV_32F);
	pHomo.at<float>(0,0) = p.x;
	pHomo.at<float>(1,0) = p.y;
	pHomo.at<float>(2,0) = p.z;
	pHomo.at<float>(3,0) = 1.0;

	pHomoRes = invFK * pHomo;
	cout << "inv      : " << invFK << endl;
	cout << "pHomo    : " << pHomo << endl;
	cout << "pHomeCalc: " << pHomoRes << endl << endl;

	pCamCoordSys.x = pHomoRes.at<float>(0,0);
	pCamCoordSys.y = pHomoRes.at<float>(1,0);
	pCamCoordSys.z = pHomoRes.at<float>(2,0);

	return (cam_->getPixelData(pCamCoordSys,pixel));
}
