//============================================================================================================
//!
//! \file gaussianProjection.h
//!
//! \brief gaussianProjection process file.
//!
//============================================================================================================
#ifndef __GAUSSIANPROJECTION_H
#define __GAUSSIANPROJECTION_H

#include <iostream>
#include <cmath>

namespace gp
{
	const double pi = 3.1415926535897932384626433832795;
}

typedef struct
{
	double a;
	double A1;
	double A2;
	double A3;
	double A4;
	double A5;
	double e2;
	double e12;
}eEllipCoeffType;

class gaussianProjection
{
	
public:
	/** \brief Constructor
	*/
	gaussianProjection();

	/** \brief forward projection step1
	* \param[in] Original of lat coordinate
	* \param[in] Original of lon coordinate
	*/
	void forwardProjectStep1(double latOriginal, double lonOriginal);

	/** \brief forward projection step2
	* \param[in]  lat coordinate
	* \param[in]  lon coordinate
	* \param[in]  head direction
	* \param[out] reference of x coordinate
	* \param[out] reference of y coordinate
	*/
	void forwardProjectStep2(double latInput, double lonInput, double headInput, double &xOutput, double &yOutput);

	/** \brief backward projection step1
	* \param[in] Original of lat coordinate
	* \param[in] Original of lon coordinate
	*/
	void backwardProjectStep1(double latOriginal, double lonOriginal);

	/** \brief backward projection step2
	* \param[in]  x coordinate
	* \param[in]  y coordinate
	* \param[in]  head direction
	* \param[out] reference of lat coordinate
	* \param[out] reference of lon coordinate
	*/
	void backwardProjectStep2(double xInput, double yInput, double headInput, double &latOutput, double &lonOutput);

	/** \brief set reference lon
	* \param[in]  lon coordinate
	*/
	void setReferenceLon(double setLon);

	/** \brief set projection parameters
	* \param[in]  parameters
	*/
	void setProjectionPara(eEllipCoeffType setPara);


private:
	/** \brief DDD to DMS
	* \param[in] DDD
	*/
	static double DDD2DMS(double DDD);

	/** \brief DMS to RAD
	* \param[in] DMS
	*/
	static double convertDms2Rad(double Dms);

	/** \brief Lat,Lon to x,y
	* \param[in]  Lat
	* \param[in]  Lon
	* \param[in]  Lon Reference
	* \param[out] reference of original x coordinate
	* \param[out] reference of original y coordinate
	*/
	void fgConvertBl2xy(double ConvB, double Convl, double CurL0, double &xOriginal, double &yOriginal);

	/** \brief x,y to Lat,Lon
	* \param[in]  x
	* \param[in]  y
	* \param[in]  Lon Reference
	* \param[out] reference of lat coordinate
	* \param[out] reference of lon coordinate
	*/
	void fgConvertxy2Bl(double Convx, double Convy, double CurL0, double &latOutput, double &lonOutput);
private:
	eEllipCoeffType eEllipCoeff;
	double lonReference;
	double CurL0;

	double centerX;
	double centerY;
};

#endif //__GAUSSIANPROJECTION_H