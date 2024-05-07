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
 * Center of the camera image is the origin of the 3-dim. Euclidean coordinate system KS0.
 * The Orientation of the camera is the positive direction of the x-axis.
 * The FOV of the camera can not be larger 170 degree.
 * Height and width pixel resolution of the camera can not be smaller 1.
 * The camera can only detected one point in the 3-dim space. The point is given
 * in the coordinate system KS0 of the camera.
 *
 * The detected point is given by an openCV image data structure and by the pixel
 * values (HxW). Details can be found in the specification of the corresponding
 * methods.
 *
 *
 */
class ICam{
public:
	virtual void    setHeight(int h) = 0;
	virtual void    setWidth(int w) = 0;
	virtual void    setFOV(int angle)  = 0;

	/**
	 *
	 *  \brief Returns the image data of the camera detecting the
	 *  point p given.
	 *
	 *  \p p 3-dimensional vector representing a point in space.
	 *  The coordinates are given in the reference frame of the camera (KS0).
	 *
	 *  \return cv::Mat openCV data image data representing the given
	 *  point p  as white circle. If the given point ist not in the
	 *  field of view, the image contains no circle ("black image only").
	 *
	 */
	virtual cv::Mat getImgData(cv::Point3d p) = 0;


	/**
	 *
	 *  \brief Returns the pixal data of the camera detecting the
	 *  point p given.
	 *
	 *  \p p 3-dimensional vector representing a point in space.
	 *  The coordinates are given in the reference frame of the camera (KS0).
	 *
	 *  \p pixel[2] (call be reference). The 2-dim array will contain the
	 *  pixel data (pixel[0] (Width) and pixel[1] (Height)) of the point detected.
	 *  If the given point p is not in the field of view then values in the array
	 *  are all -1.
	 *  If this method is called a field of size 2 has to be given a parameter.
	 *
	 */
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
	int FOVdeg_;
	float FOVrad_;

	/**
	 *
	 * \p angleFOV int
	 *
	 * \return 0 if point is not in FOV, otherwise 1
	 *
	 */
	int isPointInFOV(cv::Point3d p);

	/**
	 *
	 *
	 *
	 */
	float deg2rad(int angleDeg);


	int interSectionPointToPixelCoord(float value, int imageDim);



};



#endif /* CAM_HPP_ */
