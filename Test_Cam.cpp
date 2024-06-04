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
	float x = 2000.0;
	float limit = 6000;
	float step = limit / 2;

	Cam cam(camW, camH, 160);

	cv::Point pix;

	namedWindow("Cam", cv::WINDOW_AUTOSIZE); // Create Window
	cv::Mat src = cv::Mat::zeros(cam.getHeight(),cam.getWidth(),CV_8UC3);


	cv::Point3d v;
	for(float y = -limit; y <= limit; y+=step){
		for(float z = -limit; z <= limit; z+=step){
			v.x = x;
			v.y = y;
			v.z = z;
			src = src + cam.getImgData(v);
			cout << y << endl;
		}
	}

	imshow( "Cam", src);
	cv::waitKey(0);
	return 0;
}

