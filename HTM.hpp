#ifndef HTM_HPP_INCLUDED
#define HTM_HPP_INCLUDED

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 *
 * \class IHTM
 *
 * \brief Interface for a 4x4 HTM with 1 x rotation
 * (either rotation axis X, Y or Z) and translation
 * parameter (x,y and z).
 *
 *
 */
class IHTM{
public:
    virtual cv::Mat updateAngRot(float v) = 0;
    virtual cv::Mat updateTransX(float v) = 0;
    virtual cv::Mat updateTransY(float v) = 0;
    virtual cv::Mat updateTransZ(float v) = 0;

    virtual cv::Mat current() = 0;

    virtual float getRotAngle() = 0;
    virtual float getTransX() = 0;
    virtual float getTransY() = 0;
    virtual float getTransZ() = 0;

};


/**
 *
 * \class HTM3dTransRot abstract class
 *
 */
class HTM3dTransRot : public IHTM{
public:

	cv::Mat updateAngRot(float v);
    cv::Mat updateTransX(float v);
    cv::Mat updateTransY(float v);
    cv::Mat updateTransZ(float v);
    cv::Mat current();

    float getRotAngle();
    float getTransX();
    float getTransY();
    float getTransZ();


protected:
	cv::Mat m_;
	float angle_, transX_, transY_, transZ_;
	virtual cv::Mat createMatrix(float angle, float x, float y, float z) = 0;
};


class HTM3dTransRotX : public HTM3dTransRot{
public:
	HTM3dTransRotX(float angle=0, float x = 0, float y = 0, float z = 0);
	~HTM3dTransRotX();
protected:
	cv::Mat createMatrix(float angle, float x, float y, float z);
};


class HTM3dTransRotY : public HTM3dTransRotX{
public:
	HTM3dTransRotY(float angle=0, float x = 0, float y = 0, float z = 0);
	~HTM3dTransRotY();
protected:
	cv::Mat createMatrix(float angle, float x, float y, float z);
};

class HTM3dTransRotZ : public HTM3dTransRotX{
public:
	HTM3dTransRotZ(float angle=0, float x = 0, float y = 0, float z = 0);
	~HTM3dTransRotZ();
protected:
	cv::Mat createMatrix(float angle, float x, float y, float z);
};

IHTM *factory(char mType, float angle, float x, float y, float z);


#endif // HTM_HPP_INCLUDED
