#include "HTM.hpp"
#include "KinChain.hpp"


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>


#define PI 3.14159

#define wW 800
#define wH 800


const int alpha_slider1_max = 360;
const int alpha_slider2_max = 360;
int alpha_slider1;
int alpha_slider2;
double alpha;
double beta;
cv::Mat src1 = cv::Mat::zeros( wW, wH, CV_8UC3 );


static void on_trackbar( int, void* );

Manipulator_2D2J robot;
float joint[2];
cv::Point3d tcp;
cv::Point3d *robotPoints;

void MyFilledCircle( cv::Mat img, cv::Point3d center, cv::Scalar c );
void MyLine( cv::Mat img, cv::Point3d start, cv::Point3d end, cv::Scalar c );

int main(){

	robotPoints = new cv::Point3d[robot.getNmbRobotPoints()];


	robot.getRobotCoord(robotPoints);
	tcp = robotPoints[robot.getNmbRobotPoints() - 1];
	std::cout << tcp << std::endl;



	 alpha_slider1 = 180;
	 alpha_slider2 = 180;

	 namedWindow("2D 2J robot manipulator", cv::WINDOW_AUTOSIZE); // Create Window

	 char TrackbarName1[50];
	 char TrackbarName2[50];
	 snprintf( TrackbarName1, sizeof(TrackbarName1), "J0: 2*PI*((x-180)/%d)", alpha_slider1_max);
	 snprintf( TrackbarName2, sizeof(TrackbarName2), "J1: 2*PI*((x-180)/%d)", alpha_slider2_max);
	 cv::createTrackbar( TrackbarName1, "2D 2J robot manipulator", &alpha_slider1, alpha_slider1_max, on_trackbar );
	 cv::createTrackbar( TrackbarName2, "2D 2J robot manipulator", &alpha_slider2, alpha_slider2_max, on_trackbar );

	 on_trackbar( alpha_slider1, 0 );
	 on_trackbar( alpha_slider2, 0 );

	 cv::waitKey(0);

    return 0;
}


static void on_trackbar( int, void* )
{
	joint[0] = 2*PI*( ((float) (alpha_slider1 - 180))/alpha_slider1_max);
	joint[1] = 2*PI*( ((float)  (alpha_slider2 -180))/alpha_slider2_max );
	robot.setJointValues(joint);
	robot.getRobotCoord(robotPoints);
	tcp = robotPoints[robot.getNmbRobotPoints() - 1];
	cv::Point3d base = robotPoints[0];
	cv::Point3d j0 = robotPoints[1];
	cv::Point3d j1 = robotPoints[2];

	std::cout << tcp << std::endl;
	src1 = cv::Mat::zeros( wW, wH, CV_8UC3 );
	cv::Scalar c = cv::Scalar(255,255,255);
	MyLine(src1,base,j0,c);
	MyLine(src1,j0,j1,c);
	MyLine(src1,j1,tcp,c);
	c = cv::Scalar(255,0,0);
	MyFilledCircle(src1, tcp,c);
	MyFilledCircle(src1, base,c);
	MyFilledCircle(src1, j0,c);
	MyFilledCircle(src1, j1,c);


	imshow( "2D 2J robot manipulator", src1);
}


void MyFilledCircle( cv::Mat img, cv::Point3d center, cv::Scalar c )
{
	cv::Point p2d( (center.x+(wW/2)),wH - (center.y+(wH/3)));

	cv::circle( img, p2d, wH/100, c, cv::FILLED, cv::LINE_8 );
	return;
}


void MyLine( cv::Mat img, cv::Point3d start, cv::Point3d end, cv::Scalar c )
{
 int thickness = 2;
 int lineType = cv::LINE_8;
 cv::Point p2dStart(start.x+(wW/2),wH - (start.y+(wH/3)));
 cv::Point p2dEnd(end.x+(wW/2),wH - (end.y+(wH/3)));
 cv::line( img, p2dStart, p2dEnd, c, thickness, lineType );
 return;
}
