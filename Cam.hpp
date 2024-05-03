#ifndef CAM_HPP_
#define CAM_HPP_


/**
 *
 * \file Cam.hpp
 *
 * \brief Contains class declarations for
 * a simple camera that delivers to given 3D point
 * the pixel coordinates if the point is in the field of
 * view of the camera.
 *
 * \author Martin Huelse (c) 2024
 *
 * HSBI (email: martin.huelse@hsbi.de)
 *
 *
 */

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>




/**
 *
 * \class ICam
 *
 * \brief Interface of the camera class.
 *
 *
 */
class ICam{
public:
	virtual void    setHeight(int h) = 0;
	virtual void    setWidth(int w) = 0;
	virtual void    setFOV(int angle)  = 0;
	virtual cv::Mat getImgData(cv::Point3d p) = 0;
	virtual void    getPixelData(cv::Point3d p, int pixel[2]) = 0;

};



class Cam : public ICam{
public:
		    Cam();
		    Cam(int h, int w, int fov);
	void    setHeight(int h);
	void    setWidth(int w);
	void    setFOV(int angle);
	cv::Mat getImgData(cv::Point3d p);
	void    getPixelData(cv::Point3d p, int pixel[2]);

protected:
	int height_;
	int width_;
	int FOV_;


private:
	void   insersectionObjectRayVirtualScreen(double *y, double *z);
	double maxYvalueVirtualScreen(cv::Point3d p);
	double minYvalueVirtualScreen(cv::Point3d p);
	double maxZvalueVirtualScreen(cv::Point3d p);
	double minZvalueVirtualScreen(cv::Point3d p);


};



#endif /* CAM_HPP_ */
