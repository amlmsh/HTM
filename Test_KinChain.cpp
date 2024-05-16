/*
 * Test_KinChain.cpp
 *
 *  Created on: May 8, 2024
 *      Author: ubuntu
 */


#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "KinChain.hpp"

#define PI 3.14159

using namespace std;

void Test_PanTilt();
void Test_PanTiltActiveVis();

int main(){

	//Test_PanTilt();

	Test_PanTiltActiveVis();

	return 0;
}


void Test_PanTilt(){
	float j[2];
	cv::Point3d p[3];

	PanTilt pt;


	do{
		std::cout << "pan :"; std::cin >> j[0];
		std::cout << "tilt:"; std::cin >> j[1];
		pt.setJointValues(j);
		pt.getRobotCoord(p);
		cout << "TCP: " <<p[2] << endl;
		cout << pt.getForwardKin() << endl;
	}while(1);

	return;
}


void Test_PanTiltActiveVis(){

	// camera is fixed, objects (points) are located at different places
	PanTiltActiveVisionSystem ptActVis;

	float p,t;
	int pixel[2];
	cv::Point3d v;
	v.x = 3;
	v.y = 0;
	v.z = 0;



	namedWindow("PanTiltActVis", cv::WINDOW_AUTOSIZE); // Create Window
	cv::Mat src = cv::Mat::zeros(ptActVis.getHeight(),ptActVis.getWidth(),CV_8UC3);

	ptActVis.getPixelData(v,pixel);
	src = src + ptActVis.getImgData(v);
	cout << pixel[0] << ",  " << pixel[1] << endl;

	v.y = 10.5;
	v.z = 10.5;
	ptActVis.getPixelData(v,pixel);
	src = src + ptActVis.getImgData(v);
	cout << pixel[0] << ",  " << pixel[1] << endl;

	v.y = -10.5;
	v.z = -10.5;
	ptActVis.getPixelData(v,pixel);
	src = src + ptActVis.getImgData(v);
	cout << pixel[0] << ",  " << pixel[1] << endl;


	imshow( "PanTiltActVis", src);
	cv::waitKey(0);

	return;
}
