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



int main(){

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
	cout << pixel[0] << ",  " << pixel[1] << endl;

	v.y = 2;
	v.z = 2;
	ptActVis.getPixelData(v,pixel);
	cout << pixel[0] << ",  " << pixel[1] << endl;

	v.y = -2;
	v.z = -2;
	ptActVis.getPixelData(v,pixel);
	cout << pixel[0] << ",  " << pixel[1] << endl;


	imshow( "PanTiltActVis", src);
	cv::waitKey(0);

	return 0;
}
