#include "Cam.hpp"


#include <string>


Cam::Cam(){
	height_ = 480;
	width_ = 640;
	FOV_ = 90;
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
	FOV_ = fov;
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
	FOV_ = angle;
	return;
}

cv::Mat getImgData(cv::Point3d p){
	throw std::string("nyi");
}
void    getPixelData(cv::Point3d p, int pixel[2]){
	throw std::string("nyi");
}

