#include "Cam.hpp"


#include <string>
#include <cmath>

Cam::Cam(){
	height_ = 480;
	width_ = 640;
	FOVdeg_ = 90;
	FOVrad_ = deg2rad(FOVdeg_);
	return;
}


Cam::Cam(int h, int w, int fov){
	if((h < 1) || (w < 1) || (fov < 1)){
		throw std::string("constructor class Cam: all parameter must not be smaller than 1.");
	};

	if(fov > 170){
		throw std::string("constructor class Cam: FOV parameter too large (range must fit [1...170]).");
	}

	height_ = h;
	width_ = w;
	FOVdeg_ = fov;
	FOVrad_ = deg2rad(FOVdeg_);
	return;
}


void Cam::setHeight(int h){
	if(h < 1 ){
		throw std::string("method setHeight class Cam: all parameter must not be smaller than 1.");
	};
	height_ = h;
	return;
}

void Cam::setWidth(int w){
	if(w < 1 ){
		throw std::string("method setWidth class Cam: all parameter must not be smaller than 1.");
	};
	width_ = w;
	return;
}

void Cam::setFOV(int angle){
	if(angle < 1 || angle > 170 ){
		throw std::string("method setFOV class Cam: FOV range violated ([1...170]).");
	};
	FOVdeg_ = angle;
	FOVrad_ = deg2rad(FOVdeg_);
	return;
}

cv::Mat Cam::getImgData(cv::Point3d p){
	throw std::string("nyi");
}



int Cam::isPointInFOV(cv::Point3d p){
	float maxPosFOV = tan(FOVrad_/2.0);
	if( fabs(p.y / p.x) > maxPosFOV) return 0;
	if( fabs(p.z / p.x) > maxPosFOV) return 0;
	return 1;
}

float Cam::deg2rad(int angleDeg){
	return ( (3.1415926*((float) angleDeg)) / 180.0);
}


int Cam::interSectionPointToPixelCoord(float value, int imageDim){
	float tmp1, tmp2;

	tmp1 = ((float)(imageDim - 1) / 2.0);
	tmp2 = ((-value)/(tan(FOVrad_*0.5))) + 1.0;

	int pixelValue = ((tmp1*tmp2)+0.5);

	if(pixelValue < 0) pixelValue = 0;
	if(pixelValue > (imageDim-1)) pixelValue = imageDim - 1;

	return pixelValue;
}


void   Cam::getPixelData(cv::Point3d p, int pixel[2]){

	if(isPointInFOV(p) == 0){
		pixel[0] = -1;
		pixel[1] = -1;
	}

	float intSecPointY, intSecPointZ;
	intSecPointY = p.y / p.x;
	intSecPointZ = p.z / p.x;

	pixel[0] = interSectionPointToPixelCoord(intSecPointY, width_);
	pixel[1] = interSectionPointToPixelCoord(intSecPointZ, height_);

	return;
}




