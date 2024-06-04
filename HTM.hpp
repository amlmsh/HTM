#ifndef HTM_HPP_INCLUDED
#define HTM_HPP_INCLUDED

/**
 *
 * \mainpage HTM (Homogeneous Transformation Matrixes for Robotic Manipulators)
 *
 * \author Martin Huelse (c) 2024
 *
 * HSBI (email: martin.huelse@hsbi.de)
 *
 * \section  Introduction
 * An C++ object oriented framework is provided
 * that shall help to build up forward kinematics
 * for 3D kinematic chains.
 * The transformation matrixes include rotation and
 * translation only.
 *
 * The matrix implementation makes use of the openCV
 * data structure 'Mat', algorithms, graphics and GUI
 * elements are also based on openCV libraries.
 *
 */


/**
 *
 * \file HTM.hpp
 *
 * \brief Contains class declarations for
 * homogeneous transformation matrixes implementation.
 *
 *
 *
 */



#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 *
 * \class IHTM
 *
 * \brief Interface for either a 4x4 HTM with 1 x rotation
 * (either rotation axis X, Y or Z) and 1 x translation
 * parameters (x,y and z) OR HTM defined by DH (Denavit-Hartenberg)
 * parameter.
 *
 *
 */
class IHTM{
public:
	virtual ~IHTM(){};

	/**
	 *
	 *  The HTM based on the new rotation angle is
	 *  calculated and returned.
	 *	\p v given in rad (angle value)
	 *  \return cv::Mat (openCV data structure for representing HTM-matrix)
	 *
	 */
    virtual cv::Mat updateAngRot(float v) = 0;

    /**
	 *
	 *  The HTM is calculated and returned. The calculation is based
	 *  on the new x-component value of the 3D
	 *  translation vector.
	 *	\p v given in unit
	 *  \return cv::Mat (openCV data structure for representing HTM-matrix)
	 *
	 */
    virtual cv::Mat updateTransX(float v) = 0;


    /**
	 *
	 *  The HTM is calculated and returned. The calculation is based
	 *  on the new y-component value of the 3D
	 *  translation vector.
	 *	\p v given in unit
	 *  \return cv::Mat (openCV data structure for representing HTM-matrix)
	 *
	 */
    virtual cv::Mat updateTransY(float v) = 0;

    /**
	 *
	 *  The HTM is calculated and returned. The calculation is based
	 *  on the new z-component value of the 3D
	 *  translation vector.
	 *	\p v given in unit
	 *  \return cv::Mat (openCV data structure for representing HTM-matrix)
	 *
	 */
    virtual cv::Mat updateTransZ(float v) = 0;

    /**
	 *
	 *  The current HTM is returned.
	 *  \return cv::Mat (openCV data structure for representing HTM-matrix)
	 *
	 */
    virtual cv::Mat current() = 0;

    /**
	 *
	 *  Returns the rotation angle the current HTM is calculated with.
	 *  Notice! Each HTM represents only one rotation. The rotation
	 *  axis can either be the X-axis, Y-axis or Z-axis.
	 *  \return float (rad) rotation angle.
	 *
	 */
    virtual float getRotAngle() = 0;


    /**
	 *
	 *  Returns the x-component of the translation vector
	 *  the current HTM is calculated with.
	 *  \return float (unit) x-component of the translation.
	 *
	 */
    virtual float getTransX() = 0;

    /**
	 *
	 *  Returns the x-component of the translation vector
	 *  the current HTM is calculated with.
	 *  \return float (unit) x-component of the translation.
	 *
	 */
    virtual float getTransY() = 0;

    /**
	 *
	 *  Returns the x-component of the translation vector
	 *  the current HTM is calculated with.
	 *  \return float (unit) x-component of the translation.
	 *
	 */
    virtual float getTransZ() = 0;


    /**
     *
     * Returns the HTM defined by the given DH-parameter.
     *
     * \p phi float DH-parameter phi
     * \p alpha float DH-parameter alpha
     * \p a float DH-parameter a
     * \p d float DH-parameter d
     * \return cv::Mat (openCV data structure for representing HTM-matrix)
     *
     */
    virtual cv::Mat updateDHparam(float phi, float alpha, float a, float d) = 0;

};


/**
 *
 * \class HTM3dTransRot
 *
 *
 * \brief Implements basic functionality of the HTM
 * definition and calculations.
 * It is an abstract class and needs to be used a
 * parents class for implementing the HTMs for
 * the rotation of either X-, Y- or Z-axis.
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

    /**
     *
     * \brief Data type from openCV representing a matrix
     *
     */
	cv::Mat m_;

	/**
	 *
	 * \brief parameter representing 1 x rotation(angle in rad) and
	 * translation (x,y,z-components of the translation vector)
	 *
	 */
	float angle_, transX_, transY_, transZ_;

	/**
	 *
	 * \brief Calculation and storing the new HTM based on the given
	 * parameters for rotation and translation.
	 * For specific HTMs you must overwrite this methods in the
	 * corresponding child class.
	 *
	 *
	 * \p angle float angle (rad) of rotation
	 * \p x float (unit) x-component of the translation vector
	 * \p y float (unit) y-component of the translation vector
	 * \p y float (unit) z-component of the translation vector
	 * \return cv::Mat openCV data type representing the calculated HTM
	 *
	 */
	virtual cv::Mat createMatrix(float angle, float x, float y, float z) = 0;
};

/**
 *
 * \class HTM3dTransRotX
 *
 * \brief Implements the HTM generation of 1 x translation
 * and 1 x rotation, where the X-axis is the rotation axis.
 *
 */
class HTM3dTransRotX : public HTM3dTransRot{
public:

	/**
	 *
	 * Constructor, parameterized
	 *
	 */
	HTM3dTransRotX(float angle=0, float x = 0, float y = 0, float z = 0);

	/**
	 *
	 * Destructor
	 *
	 */
	~HTM3dTransRotX();

protected:

	/**
	 *
	 * \brief Calculation and storing the new HTM based on the given
	 * parameters for rotation (rotation axis is X-axis) and translation.
	 *
	 * \p angle float angle (rad) of rotation
	 * \p x float (unit) x-component of the translation vector
	 * \p y float (unit) y-component of the translation vector
	 * \p y float (unit) z-component of the translation vector
	 * \return cv::Mat openCV data type representing the calculated HTM
	 *
	 */
	cv::Mat createMatrix(float angle, float x, float y, float z);

	/**
	 *
	 * Not implemented.
	 * Throws an exception (string).
	 *
	 */
	cv::Mat updateDHparam(float phi, float alpha, float a, float d);
};

/**
 *
 * \class HTM3dTransRotY
 *
 * \brief Implements the HTM generation of 1 x translation
 * and 1 x rotation, where the Y-axis is the rotation axis.
 *
 */
class HTM3dTransRotY : public HTM3dTransRotX{
public:

	/**
	 *
	 * Constructor, parameterized
	 *
	 */
	HTM3dTransRotY(float angle=0, float x = 0, float y = 0, float z = 0);

	/**
	 *
	 * Destructor
	 *
	 */
	~HTM3dTransRotY();
protected:

	/**
	 *
	 * \brief Calculation and storing the new HTM based on the given
	 * parameters for rotation (rotation axis is Y-axis) and translation.
	 *
	 * \p angle float angle (rad) of rotation
	 * \p x float (unit) x-component of the translation vector
	 * \p y float (unit) y-component of the translation vector
	 * \p y float (unit) z-component of the translation vector
	 * \return cv::Mat openCV data type representing the calculated HTM
	 *
	 */
	cv::Mat createMatrix(float angle, float x, float y, float z);


};


/**
 *
 * \class HTM3dTransRotZ
 *
 * \brief Implements the HTM generation of 1 x translation
 * and 1 x rotation, where the Z-axis is the rotation axis.
 *
 */
class HTM3dTransRotZ : public HTM3dTransRotX{
public:

	/**
	 *
	 * Constructor, parameterized
	 *
	 */
	HTM3dTransRotZ(float angle=0, float x = 0, float y = 0, float z = 0);

	/**
	 *
	 * Destructor
	 *
	 */
	~HTM3dTransRotZ();
protected:

	/**
	 *
	 * \brief Calculation and storing the new HTM based on the given
	 * parameters for rotation (rotation axis is Z-axis) and translation.
	 *
	 * \p angle float angle (rad) of rotation
	 * \p x float (unit) x-component of the translation vector
	 * \p y float (unit) y-component of the translation vector
	 * \p y float (unit) z-component of the translation vector
	 * \return cv::Mat openCV data type representing the calculated HTM
	 *
	 */
	cv::Mat createMatrix(float angle, float x, float y, float z);


};


class HTM3dDH : public IHTM{
public:
	HTM3dDH();
	~HTM3dDH();
    cv::Mat updateAngRot(float v){throw std::string("Not supported in this class.");};
    cv::Mat updateTransX(float v){throw std::string("Not supported in this class.");};
    cv::Mat updateTransY(float v){throw std::string("Not supported in this class.");};
    cv::Mat updateTransZ(float v){throw std::string("Not supported in this class.");};
    cv::Mat current();
    float   getRotAngle(){throw std::string("Not supported in this class.");};
    float   getTransX(){throw std::string("Not supported in this class.");};
    float   getTransY(){throw std::string("Not supported in this class.");};
    float   getTransZ(){throw std::string("Not supported in this class.");};
    cv::Mat updateDHparam(float phi, float alpha, float a, float d);
protected:
    /**
     *
     * \brief Data type from openCV representing a matrix
     *
     */
	cv::Mat m_;
};


/**
 *
 * \brief Factory function generating an object that is representing
 * a HTM with one rotation and one translation part.
 * The first first parameter determines the rotation axis (X-, Y- or
 * Z-axis.
 *
 * \p mType char defines the rotation axis. values can be
 *          'X', 'Y' or 'Z' other values result in the unity matrix.
 * \p angle float rotation angle (rad).
 * \p x float x-component of the translation vector
 * \p y float y-component of the translation vector
 * \p z float z-component of the translation vector
 *
 */
IHTM *factory(char mType, float angle, float x, float y, float z);


#endif // HTM_HPP_INCLUDED
