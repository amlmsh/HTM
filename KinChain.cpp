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


PanTilt_DH::PanTilt_DH(){
	panValueRad_ = 0;
	tiltValueRad_ = 0;
	link0_ = 0;
	link1_ = 0;
	this->buildRobot();
	return;
}

PanTilt_DH::~PanTilt_DH(){
	return;
}


unsigned int PanTilt_DH::getPanIdx(){return 0;};
unsigned int PanTilt_DH::getTiltIdx(){return 1;};

int  PanTilt_DH::getNmbJoints(){
	return 2;
}

int  PanTilt_DH::getNmbRobotPoints(){
	return 3;
}

void PanTilt_DH::getRobotCoord(cv::Point3d p[]){
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

void PanTilt_DH::setTCPCoord(cv::Point3d){
	throw std::string("not implemented.");
}

void PanTilt_DH::getJointValues(float j[]){
	j[this->getPanIdx()] = panValueRad_;
	j[this->getTiltIdx()] = tiltValueRad_;
	return;
}

void PanTilt_DH::setJointValues(const float j[]){
	panValueRad_ = j[this->getPanIdx()];
	tiltValueRad_ = j[this->getTiltIdx()];
	this->buildRobot();
	return;
}


void PanTilt_DH::buildRobot(){
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


cv::Mat PanTilt_DH::getForwardKin(){
	return T_2_0_;

}

cv::Mat PanTilt_DH::getInvertedForwardKin(){
	return T_0_2_;
}





PanTiltActiveVisionSystem::PanTiltActiveVisionSystem(int width, int height, int FOV, char c){
	if((c != 'H') && (c != 'D')){
		throw string("Type-of-camera parameter is not correct. Please check documentation.");
	}

	if(c == 'H'){
		panTilt_ = new PanTilt_HTM();
	}else{
		panTilt_ = new PanTilt_DH();
	}

	try{
		cam_ = new Cam(width, height, FOV);
	}catch(string msg){
		string s("Constructor PanTilActiveVisionSystem::");
		s = s + msg;
		throw s;
	}catch(...){
		string s("Constructor PanTilActiveVisionSystem::unknown error.");
		throw s;
	}
	return;
}

PanTiltActiveVisionSystem::~PanTiltActiveVisionSystem(){
	delete panTilt_;
	delete cam_;
	return;
}


void    PanTiltActiveVisionSystem::setPan(float angleRad){
	float valuesRad[panTilt_->getNmbJoints()];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->getPanIdx()] = angleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}

void    PanTiltActiveVisionSystem::setTilt(float angleRad){
	float valuesRad[panTilt_->getNmbJoints()];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->getTiltIdx()] = angleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}


void    PanTiltActiveVisionSystem::setDeltaPan(float deltaAngleRad){
	float valuesRad[panTilt_->getNmbJoints()];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->getPanIdx()] += deltaAngleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}

void    PanTiltActiveVisionSystem::setDeltaTilt(float deltaAngleRad){
	float valuesRad[panTilt_->getNmbJoints()];
	panTilt_->getJointValues(valuesRad);
	valuesRad[panTilt_->getTiltIdx()] += deltaAngleRad;
	panTilt_->setJointValues(valuesRad);
	return;
}

float   PanTiltActiveVisionSystem::getPan(){
	float valuesRad[panTilt_->getNmbJoints()];
	panTilt_->getJointValues(valuesRad);
	return valuesRad[panTilt_->getPanIdx()];
}

float   PanTiltActiveVisionSystem::getTilt(){
	float valuesRad[panTilt_->getNmbJoints()];
	panTilt_->getJointValues(valuesRad);
	return valuesRad[panTilt_->getTiltIdx()];
}

int     PanTiltActiveVisionSystem::getWidth(){
	return cam_->getWidth();
}


int     PanTiltActiveVisionSystem::getHeight(){
	return cam_->getHeight();
}

cv::Mat PanTiltActiveVisionSystem::getImgData(cv::Point3d p){
	cv::Point3d pCamCoordSys;
	pCamCoordSys = this->calcCoordInCamFrame(p);
	return (cam_->getImgData(pCamCoordSys));
}

void    PanTiltActiveVisionSystem::getPixelData(cv::Point3d p, int pixel[2]){
	cv::Point3d pCamCoordSys;
	pCamCoordSys = this->calcCoordInCamFrame(p);
	return (cam_->getPixelData(pCamCoordSys,pixel));
}

cv::Point3d PanTiltActiveVisionSystem::calcCoordInCamFrame(cv::Point3d p){
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

	return pCamCoordSys;
}



PanTilt_HTM::PanTilt_HTM(){
	panValueRad_ = 0.0;
	tiltValueRad_ = 0.0;
	link0_ = 0.0;
	link1_ = 0.0;

	// define robot inital state
	linkLenght_[0] = 0;
	linkLenght_[1] = 0;
	linkLenght_[2] = link0_;
	linkLenght_[3] = link1_;

	jValue_[0] = panValueRad_;
	jValue_[1] = -0.5*3.14159; // const -PI/2
	jValue_[2] = tiltValueRad_;
	jValue_[3] = 0.5*3.14159; // const PI/2


	buildRobot();

	return;
}


unsigned int PanTilt_HTM::getPanIdx(){return 0;};
unsigned int PanTilt_HTM::getTiltIdx(){return 2;};

PanTilt_HTM::~PanTilt_HTM(){
	if(this->T_1_0_ != NULL) delete this->T_1_0_;
	if(this->T_2_1_ != NULL) delete this->T_2_1_;
	if(this->T_3_2_ != NULL) delete this->T_3_2_;
	if(this->T_4_3_ != NULL) delete this->T_4_3_;
	return;
}


int  PanTilt_HTM::getNmbJoints(){
	return this->nmbJoints_;
}

int  PanTilt_HTM::getNmbRobotPoints(){
	return this->nmbRobotPoints_;
}


cv::Mat PanTilt_HTM::getForwardKin(){
	return forwardKin_;
}

cv::Mat PanTilt_HTM::getInvertedForwardKin(){
	cv::invert(forwardKin_,invForwardKin_);
	return invForwardKin_;
}


void PanTilt_HTM::getRobotCoord(cv::Point3d points[]){
	for(int i=0; i < nmbRobotPoints_; i++){
		points[i] = jCoord_[i];
	}
	return;
}

void PanTilt_HTM::getJointValues(float v[]){
	v[this->getPanIdx()] = jValue_[this->getPanIdx()];
	v[this->getTiltIdx()] = jValue_[this->getTiltIdx()];
	return;
}

void PanTilt_HTM::setJointValues(const float v[]){
	jValue_[this->getPanIdx()] = v[this->getPanIdx()];
	jValue_[this->getTiltIdx()] = v[this->getTiltIdx()];
	this->buildRobot();
	return;
}

void PanTilt_HTM::setTCPCoord(cv::Point3d){
	throw string("NYI");
}

void PanTilt_HTM::buildRobot(){
	if(this->T_1_0_ != NULL) delete this->T_1_0_;
	if(this->T_2_1_ != NULL) delete this->T_2_1_;
	if(this->T_3_2_ != NULL) delete this->T_3_2_;
	if(this->T_4_3_ != NULL) delete this->T_4_3_;


	// define matrices
	T_1_0_ = factory('Z',-jValue_[0],0,0,0);
	T_2_1_ = factory('X',jValue_[1],0,0,0);
	T_3_2_ = factory('Z',-jValue_[2],0,linkLenght_[2],0);
	T_4_3_ = factory('X',jValue_[3],linkLenght_[3],0,0);

	// calculate final matrices
	cv::Mat T_2_0 = T_1_0_->current() * T_2_1_->current();
	cv::Mat T_3_0 = T_2_0  * T_3_2_->current();
	forwardKin_ = T_3_0  * T_4_3_->current();

	// read joint coordinates (incl. TCP)
	// base
	jCoord_[0] = cv::Point3d(0,0,0);

	//
	jCoord_[1] = cv::Point3d(T_1_0_->current().at<float>(0,3),
							 T_1_0_->current().at<float>(1,3),
							 T_1_0_->current().at<float>(2,3));

	//
	jCoord_[2] = cv::Point3d(T_2_0.at<float>(0,3),
							 T_2_0.at<float>(1,3),
							 T_2_0.at<float>(2,3));

	//
	jCoord_[3] = cv::Point3d(T_3_0.at<float>(0,3),
							 T_3_0.at<float>(1,3),
							 T_3_0.at<float>(2,3));

	//
	jCoord_[4] = cv::Point3d(forwardKin_.at<float>(0,3),
							 forwardKin_.at<float>(1,3),
							 forwardKin_.at<float>(2,3));

	return;
}



