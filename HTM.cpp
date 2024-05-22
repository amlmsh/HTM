#include "HTM.hpp"

#include <string>
#include <iostream>


IHTM *factory(char mType, float angle, float x, float y, float z){
	IHTM *m;
	switch(mType){
		case 'X': m = new HTM3dTransRotX(angle, x,y,z); break;
		case 'Y': m = new HTM3dTransRotY(angle, x,y,z); break;
		case 'Z': m = new HTM3dTransRotZ(angle, x,y,z); break;
		default:  m = new HTM3dTransRotX(0,0,0,0);
	}
	return m;
}



float HTM3dTransRot::getRotAngle(){return angle_;}
float HTM3dTransRot::getTransX()  {return transX_;}
float HTM3dTransRot::getTransY()  {return transY_;}
float HTM3dTransRot::getTransZ()  {return transZ_;}


cv::Mat HTM3dTransRot::updateAngRot(float v){
	this->angle_ = v;
	this->m_ = this->createMatrix(angle_,transX_,transY_,transZ_);
	return m_;
}

cv::Mat HTM3dTransRot::updateTransX(float v){
	this->transX_ = v;
	this->m_ = this->createMatrix(angle_,transX_,transY_,transZ_);
	return m_;
}
cv::Mat HTM3dTransRot::updateTransY(float v){
	this->transY_ = v;
	this->m_ = this->createMatrix(angle_,transX_,transY_,transZ_);
	return m_;
}
cv::Mat HTM3dTransRot::updateTransZ(float v){
	this->transZ_ = v;
	this->m_ = this->createMatrix(angle_,transX_,transY_,transZ_);
	return m_;
}

cv::Mat HTM3dTransRot::current(){
	return m_;
}


HTM3dTransRotX::HTM3dTransRotX(float angle, float x, float y, float z){
	this->angle_ = angle;
	this->transX_ = x;
	this->transY_ = y;
	this->transZ_ = z;
	m_ = this->createMatrix(this->angle_,this->transX_,this->transY_,this->transZ_);
	return;
}


HTM3dTransRotX::~HTM3dTransRotX(){
	return;

}

cv::Mat HTM3dTransRotX::createMatrix(float angle, float x, float y, float z){
	this->angle_ = angle;
	this->transX_ = x;
	this->transY_ = y;
	this->transZ_ = z;
	this->m_ = cv::Mat::eye(4,4,CV_32F);
	this->m_.at<float>(1,1) =  cos(this->angle_);
	this->m_.at<float>(1,2) =  sin(this->angle_);
	this->m_.at<float>(2,2) =  cos(this->angle_);
	this->m_.at<float>(2,1) = -sin(this->angle_);
	this->m_.at<float>(0,3) =  this->transX_;
	this->m_.at<float>(1,3) =  this->transY_;
	this->m_.at<float>(2,3) =  this->transZ_;
	return this->m_;
}


HTM3dTransRotY::HTM3dTransRotY(float angle, float x, float y, float z): HTM3dTransRotX(angle,x,y,z){
	this->m_ = this->createMatrix(this->angle_,this->transX_,this->transY_,this->transZ_);
}
HTM3dTransRotY::~HTM3dTransRotY(){
	return;
}

cv::Mat HTM3dTransRotY::createMatrix(float angle, float x, float y, float z){
	this->angle_ = angle;
	this->transX_ = x;
	this->transY_ = y;
	this->transZ_ = z;
	this->m_ = cv::Mat::eye(4,4,CV_32F);
	this->m_.at<float>(0,0) =  cos(this->angle_);
	this->m_.at<float>(0,2) = -sin(this->angle_);
	this->m_.at<float>(2,2) =  cos(this->angle_);
	this->m_.at<float>(2,0) =  sin(this->angle_);
	this->m_.at<float>(0,3) =  this->transX_;
	this->m_.at<float>(1,3) =  this->transY_;
	this->m_.at<float>(2,3) =  this->transZ_;
	return this->m_;
}

HTM3dTransRotZ::HTM3dTransRotZ(float angle, float x, float y, float z): HTM3dTransRotX(angle,x,y,z){
	this->m_ = this->createMatrix(this->angle_,this->transX_,this->transY_,this->transZ_);
}
HTM3dTransRotZ::~HTM3dTransRotZ(){
	return;
}

cv::Mat HTM3dTransRotZ::createMatrix(float angle, float x, float y, float z){
	this->angle_ = angle;
	this->transX_ = x;
	this->transY_ = y;
	this->transZ_ = z;
	this->m_ = cv::Mat::eye(4,4,CV_32F);
	this->m_.at<float>(0,0) =  cos(this->angle_);
	this->m_.at<float>(0,1) =  sin(this->angle_);
	this->m_.at<float>(1,1) =  cos(this->angle_);
	this->m_.at<float>(1,0) = -sin(this->angle_);
	this->m_.at<float>(0,3) =  this->transX_;
	this->m_.at<float>(1,3) =  this->transY_;
	this->m_.at<float>(2,3) =  this->transZ_;
	return this->m_;
}


cv::Mat HTM3dTransRotX::updateDHparam(float phi, float alpha, float a, float d){
	std::string msg;
	msg += std::string("DH-Parameter not supported in this class.");
	throw msg;
	return cv::Mat::eye(4,4,CV_32F);
}


HTM3dDH::HTM3dDH(){
	this->m_ = cv::Mat::eye(4,4,CV_32F);
}

HTM3dDH::~HTM3dDH(){
	return;
}

cv::Mat HTM3dDH::current(){
	return this->m_;
}

cv::Mat HTM3dDH::updateDHparam(float phi, float alpha, float a, float d){
	this->m_ = cv::Mat::eye(4,4,CV_32F);
	this->m_.at<float>(0,0) =  cos(phi);
	this->m_.at<float>(0,1) = -sin(phi)*cos(alpha);
	this->m_.at<float>(0,2) =  sin(phi)*sin(alpha);
	this->m_.at<float>(0,3) =  a*cos(phi);

	this->m_.at<float>(1,0) =  sin(phi);
	this->m_.at<float>(1,1) =  cos(phi)*cos(alpha);
	this->m_.at<float>(1,2) = -cos(phi)*sin(alpha);
	this->m_.at<float>(1,3) =  a*sin(phi);

	this->m_.at<float>(2,0) =  0;
	this->m_.at<float>(2,1) =  sin(alpha);
	this->m_.at<float>(2,2) =  cos(alpha);
	this->m_.at<float>(2,3) =  d;

	return this->m_;
}



