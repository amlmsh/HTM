/*
 * Test_Cam.cpp
 *
 *  Created on: May 7, 2024
 *      Author: ubuntu
 */

#include "Cam.hpp"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace std;


int main(){
	int camW = 500;
	int camH = 250;

	Cam c(camW, camH, 55);
	float x,y,z;
	cv::Point3d v;
	cv::Point pix;
	int pixel[2];


	namedWindow("Cam", cv::WINDOW_AUTOSIZE); // Create Window
	cv::Mat src = cv::Mat::zeros(c.getHeight(),c.getWidth(),CV_8UC3);

	x = 1.0;
	for(y=-3.14; y < 3.14; y=y+0.1){
		z = y;
		v.x=x;
		v.y=sin(y);
		v.z=cos(y);
		c.getPixelData(v,pixel);
		src =  src + c.getImgData(v);
	}

	imshow( "Cam", src);
	cv::waitKey(0);

	return 0;
}

