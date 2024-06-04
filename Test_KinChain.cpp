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

void Test_PanTilt(char c);
void Test_PanTiltActiveVis(char c);
void Test_PanTiltActiveVis1(char c);
void Test_PanTiltActiveVis2(char c);
void Test_PanTiltActiveVis3(char c);


static void on_trackbarVis2CamA_Pan( int, void* );
static void on_trackbarVis2CamA_Tilt( int, void* );


static void on_trackbarVis3_Pan( int, void* );
static void on_trackbarVis3_Tilt( int, void* );


void createWorld(PanTiltActiveVisionSystem *ptAV);


const int camW = 300;
const int camH = camW;

cv::Mat src = src.zeros(camH, camW, CV_8UC3 );


int main(){
	char c;

	c = 'D';

	try{
		//Test_PanTilt(c);
		//Test_PanTiltActiveVis(c);
		//Test_PanTiltActiveVis1(c);
		//Test_PanTiltActiveVis2(c);
		Test_PanTiltActiveVis3(c);
	}catch(string msg){
		std::cout << "Error: " << msg;
	}catch(...){
		std::cout << "Error: unknown error.";
	}

	return 0;
}



void Test_PanTiltActiveVis3(char c){
	PanTiltActiveVisionSystem pt(camW,camH,170, c);

	int pan_slider  = 180;
	int tilt_slider = 180;
	int pan_slider_max = 360;
	int tilt_slider_max = 360;

	namedWindow("view world", cv::WINDOW_AUTOSIZE); // Create Window
	cv::createTrackbar( "PAN", "view world", &pan_slider,  pan_slider_max,  on_trackbarVis3_Pan, &pt );
    cv::createTrackbar( "TILT","view world", &tilt_slider, tilt_slider_max, on_trackbarVis3_Tilt, &pt );

	cv::waitKey(0);
	return;
}


static void on_trackbarVis3_Pan( int newValue, void* pt){
	((PanTiltActiveVisionSystem*)pt)->setPan( ((((float)newValue)*PI)/(float)180) - PI); // rad = newValue*PI/180 - PI
	createWorld((PanTiltActiveVisionSystem*) pt);
	imshow( "view world", src);
	return;
}

static void on_trackbarVis3_Tilt( int newValue, void* pt){
	((PanTiltActiveVisionSystem*)pt)->setTilt( ((((float)newValue)*PI)/(float)180) - PI); // rad = newValue*PI/180 - PI
	createWorld((PanTiltActiveVisionSystem*) pt);
	imshow( "view world", src);
	return;
}

void createWorld(PanTiltActiveVisionSystem *ptAV){
	float x = 1000.0;
	float limit = 6000;
	float step = limit / 7;

	src = src.zeros(camH, camW, CV_8UC3 );
	cv::Point3d v;
	for(float y = -limit; y < limit; y+=step){
		for(float z = -limit; z < limit; z+=step){
			v.x = x;
			v.y = y;
			v.z = z;
			src = src + ptAV->getImgData(v);
		}
	}
	return;
}


void Test_PanTiltActiveVis2(char c){

	PanTiltActiveVisionSystem pt(camW,camH,90, c);

	int pan_slider  = 180;
	int tilt_slider = 180;
	int pan_slider_max = 360;
	int tilt_slider_max = 360;





	 namedWindow("active cam A", cv::WINDOW_AUTOSIZE); // Create Window
	 cv::createTrackbar( "PAN", "active cam A", &pan_slider,  pan_slider_max,  on_trackbarVis2CamA_Pan, &pt );
	 cv::createTrackbar( "TILT","active cam A", &tilt_slider, tilt_slider_max, on_trackbarVis2CamA_Tilt, &pt );

	 cout << "pan:" << pan_slider << "  tilt: " << tilt_slider << endl;

	 cv::waitKey(0);
	 return;
}

static void on_trackbarVis2CamA_Pan( int newValue, void* panTiltActVis){
	PanTiltActiveVisionSystem *pt;
	pt = (PanTiltActiveVisionSystem *) panTiltActVis;
	cv::Point3d v;
	v.x = 3;
	v.y = 0;
	v.z = 0;
	int pixel[2];
	pt->setPan( ((((float)newValue)*PI)/(float)180) - PI); // rad = newValue*PI/180 - PI

	cout << "[pan,tilt] = [" << pt->getPan() << ", " << pt->getTilt() << "]\n";

	src = pt->getImgData(v);
	pt->getPixelData(v,pixel);
	cout << "[X,Y] = [" << pixel[0] << ", " << pixel[1] << "]\n";

	imshow( "active cam A", src);
	return;
}

static void on_trackbarVis2CamA_Tilt( int newValue, void* panTiltActVis){
	PanTiltActiveVisionSystem *pt;
	pt = (PanTiltActiveVisionSystem *) panTiltActVis;
	cv::Point3d v;
	v.x = 3;
	v.y = 0;
	v.z = 0;
	int pixel[2];
	pt->setTilt( ((((float)newValue)*PI)/(float)180) - PI); // rad = newValue*PI/180 - PI

	cout << "[pan,tilt] = [" << pt->getPan() << ", " << pt->getTilt() << "]\n";

	src = pt->getImgData(v);
	pt->getPixelData(v,pixel);
	cout << "[X,Y] = [" << pixel[0] << ", " << pixel[1] << "]\n";

	imshow( "active cam A", src);
	return;
}



void Test_PanTilt(char c){
	float j[2];
	cv::Point3d p[3];
	PanTilt *pt;


	if(c == 'D'){
		PanTilt_DH ptDH;
		pt = & ptDH;
	}else{
		PanTilt_HTM ptHTM;
		pt = & ptHTM;
	}


	do{
		std::cout << "pan :"; std::cin >> j[0];
		std::cout << "tilt:"; std::cin >> j[1];
		pt->setJointValues(j);
		pt->getRobotCoord(p);
		cout << "TCP: " <<p[2] << endl;
		cout << pt->getForwardKin() << endl;
	}while(1);

	return;
}

void Test_PanTiltActiveVis1(char c){
	// object (point) is fixed, camera moves
	PanTiltActiveVisionSystem ptActVis(640,480,120,c);

	float p,t;
	int pixel[2];
	cv::Point3d v;
	v.x = 3;
	v.y = 0;
	v.z = 0;

	float pAbsMax = 0.01;

	namedWindow("PanTiltActVis", cv::WINDOW_AUTOSIZE); // Create Window
	cv::Mat src = cv::Mat::zeros(ptActVis.getHeight(),ptActVis.getWidth(),CV_8UC3);

	int i = 0;
	while(i < 14){
		i++;

		for(p=-pAbsMax; p< pAbsMax; p=p + (pAbsMax/100.0)){
			t = sqrt(pAbsMax*pAbsMax - p*p);
			ptActVis.setPan(p)	;
			ptActVis.setTilt(t);

			cout << p << " " << t << endl;
			ptActVis.getPixelData(v,pixel);
			src = src + ptActVis.getImgData(v);

			pAbsMax = pAbsMax + (pAbsMax/1000.0);
		}
		for(p=pAbsMax; p> -pAbsMax; p= p - (pAbsMax/100.0)){
			t = -sqrt(pAbsMax*pAbsMax - p*p);
			ptActVis.setPan(p)	;
			ptActVis.setTilt(t);
			cout << p << " " << t << endl;
			ptActVis.getPixelData(v,pixel);

			src = src + ptActVis.getImgData(v);

			pAbsMax = pAbsMax + (pAbsMax/1000.0);
		}

	}



	imshow( "PanTiltActVis", src);
	cv::waitKey(0);

	return;

}


void Test_PanTiltActiveVis(char c){

	// camera is fixed, objects (points) are located at different places
	PanTiltActiveVisionSystem ptActVis(640,480,120,c);

	int pixel[2];
	cv::Point3d v;
	v.x = 30;
	v.y = 1;
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
