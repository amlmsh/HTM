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


int main(){
	int camW = 300;
	int camH = 400;

	Cam c(camW, camH, 90);
	float x,y,z;
	cv::Point3d v;
	cv::Point pix;
	int pixel[2];


	namedWindow("Cam", cv::WINDOW_AUTOSIZE); // Create Window
	cv::Mat src = cv::Mat::zeros( camW, camH, CV_8UC3 );
	cv::Scalar color = cv::Scalar(255,255,255);

	x = 10.0;
	for(y=-2; y < 2; y=y+0.1){
		z = y;
		v.x=x;
		v.y=y;
		v.z=z;
		c.getPixelData(v,pixel);
		std::cout << "[WxH]=[" << pixel[0] << ", " << pixel[1] << "]\n";

		cv::Point pix(pixel[0],pixel[1]);

		cv::circle( src, pix, 3, color, cv::FILLED, cv::LINE_8 );
		imshow( "Cam", src);
	}


	cv::waitKey(0);


	return 0;
}

