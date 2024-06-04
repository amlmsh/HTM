/**
 *
 * \file KinChain.hpp
 *
 * \brief Contains class declarations for...
 *
 *
 *
 */
#ifndef KINCHAIN_HPP_
#define KINCHAIN_HPP_


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "HTM.hpp"
#include "Cam.hpp"


class IKinChain{
public:
	virtual 		~IKinChain(){};
	virtual int     getNmbJoints()                = 0;
	virtual int     getNmbRobotPoints()           = 0;
	virtual void    getRobotCoord(cv::Point3d[])  = 0;
	virtual void    getJointValues(float[])       = 0;
	virtual void    setJointValues(const float[]) = 0;
	virtual void    setTCPCoord(cv::Point3d)      = 0;
	virtual void    buildRobot()                  = 0;
};


class IPanTiltActiveVisionSystem{
public:
	virtual void    setPan(float angleRad) = 0;
	virtual void    setTilt(float angleRad) = 0;
	virtual void    setDeltaPan(float deltaAngleRad) = 0;
	virtual void    setDeltaTilt(float delateAngleRad) = 0;
	virtual float   getPan() = 0;
	virtual float   getTilt() = 0;
	virtual int     getWidth() = 0;
	virtual int     getHeight() = 0;
	virtual cv::Mat getImgData(cv::Point3d p) = 0;
	virtual void    getPixelData(cv::Point3d p, int pixel[2]) = 0;
};



class PanTilt : public IKinChain{
public:
	virtual ~PanTilt(){return;};
	virtual cv::Mat getForwardKin()         = 0;
	virtual cv::Mat getInvertedForwardKin() = 0;
	virtual unsigned int getPanIdx()        = 0;
	virtual unsigned int getTiltIdx()       = 0;
};



class PanTilt_DH : public PanTilt{
public:
	PanTilt_DH();
	~PanTilt_DH();
	int  getNmbJoints();
	int  getNmbRobotPoints();
	void getRobotCoord(cv::Point3d[]);
	void getJointValues(float[]);
	void setJointValues(const float[]);
	void setTCPCoord(cv::Point3d);
	void buildRobot();

	unsigned int getPanIdx();
	unsigned int getTiltIdx();

	cv::Mat getForwardKin();
	cv::Mat getInvertedForwardKin();


protected:
	HTM3dDH T_1_0_;
	HTM3dDH T_2_1_;
	cv::Mat T_2_0_;
	cv::Mat T_0_2_;

	static const int DH_PARAM_PHI_IDX_ = 0;
	static const int DH_PARAM_ALPHA_IDX_ = 1;
	static const int DH_PARAM_A_IDX_ = 2;
	static const int DH_PARAM_D_IDX_ = 3;

	float DH_Param_1_[4];
	float DH_Param_2_[4];

	float panValueRad_, tiltValueRad_, link0_, link1_;

};



class PanTilt_HTM : public PanTilt{
public:
	PanTilt_HTM();
	~PanTilt_HTM();
	int  getNmbJoints();
	int  getNmbRobotPoints();
	void getRobotCoord(cv::Point3d[]);
	void getJointValues(float[]);
	void setJointValues(const float[]);
	void setTCPCoord(cv::Point3d);
	void buildRobot();

	cv::Mat getForwardKin();
	cv::Mat getInvertedForwardKin();

	unsigned int getPanIdx();
	unsigned int getTiltIdx();


protected:
	static const int nmbJoints_ = 4;
	static const int nmbLinks_  = 4;
	static const int nmbRobotPoints_ = 5; // position base, joints and TCP

	float linkLenght_[nmbLinks_];  // links
	float jValue_[nmbJoints_]; // rotations joints values
	cv::Point3d jCoord_[nmbRobotPoints_];


	IHTM *T_1_0_ = NULL;
	IHTM *T_2_1_ = NULL;
	IHTM *T_3_2_ = NULL;
	IHTM *T_4_3_ = NULL;

	cv::Mat forwardKin_;
	cv::Mat invForwardKin_;

	float panValueRad_, tiltValueRad_, link0_, link1_;

};


class PanTiltActiveVisionSystem : public IPanTiltActiveVisionSystem{
public:
			/**
			 *
			 *  Constructor.
			 *
			 *	\p W int given in number (of pixel) horizontal (width) resolution of the camera
			 *	\p H int given in number (of pixel) vertical (height)  resolution of the camera
			 *	\p FOV int given in degree  vertical and horizontal field of view of the camera
			 *	\p c char specifies the implementation of the pan-tilt system 'D' uses DH-parameter based implementation
			 *	      'H' is the standard HTMs are used for implementing the kinematic chain of the pan-tilt system.
			 *
			 */
			PanTiltActiveVisionSystem(int width = 640, int height = 480, int FOV = 90,  char c = 'D');
			~PanTiltActiveVisionSystem();
	void    setPan(float angleRad);
	void    setTilt(float angleRad);
	void    setDeltaPan(float deltaAngleRad);
	void    setDeltaTilt(float delateAngleRad);
	float   getPan();
	float   getTilt();
	int     getWidth();
	int     getHeight();
	cv::Mat getImgData(cv::Point3d p);
	void    getPixelData(cv::Point3d p, int pixel[2]);

protected:

	PanTilt *panTilt_ = NULL;
	Cam     *cam_ = NULL;

private:
	cv::Point3d calcCoordInCamFrame(cv::Point3d p);
};

/**
 *
 * \class Manipulator_2D2J
 *
 * \brief Implementation of a 2 joint (2J) robot arm in
 * 2-dimensional space (2D).
 *
 *
 */
class Manipulator_2D2J : public IKinChain{
public:
	Manipulator_2D2J();
	~Manipulator_2D2J();
	int  getNmbJoints();
	int  getNmbRobotPoints();
	void getRobotCoord(cv::Point3d[]);
	void getJointValues(float[]);
	void setJointValues(const float[]);
	void setTCPCoord(cv::Point3d);
	void buildRobot();
protected:
	static const int nmbJoints_ = 2;
	static const int nmbLinks_  = 3;
	static const int nmbRobotPoints_ = 4; // position base, joints and TCP

	float linkLenght_[nmbLinks_];  // links
	float jValue_[nmbJoints_]; // rotations joints values
	cv::Point3d jCoord_[nmbRobotPoints_];


	IHTM *T_1_0_ = NULL;
	IHTM *T_2_1_ = NULL;
	IHTM *T_3_2_ = NULL;
};




#endif /* KINCHAIN_HPP_ */
