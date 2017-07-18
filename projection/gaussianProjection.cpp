#include "gaussianProjection.h"

/** \brief Constructor
*/
gaussianProjection::gaussianProjection()
{
	eEllipCoeff.a = 6378137;
	eEllipCoeff.A1 = 111132.9525494;
	eEllipCoeff.A2 = -16038.50840;
	eEllipCoeff.A3 = 16.83260;
	eEllipCoeff.A4 = -0.02198;
	eEllipCoeff.A5 = 0.00003;
	eEllipCoeff.e2 = 0.0066943799013;
	eEllipCoeff.e12 = 0.00673949674227;

	lonReference = 120;
	CurL0 = convertDms2Rad(lonReference);
}

/** \brief forward projection step1
* \param[in] Original of lat coordinate
* \param[in] Original of lon coordinate
*/
void gaussianProjection::forwardProjectStep1(double latOriginal, double lonOriginal)
{
	latOriginal = DDD2DMS(latOriginal);
	lonOriginal = DDD2DMS(lonOriginal);
	fgConvertBl2xy(latOriginal, lonOriginal, CurL0, centerX, centerY);
}

/** \brief forward projection step2
* \param[in]  lat coordinate
* \param[in]  lon coordinate
* \param[in]  head direction
* \param[out] reference of x coordinate
* \param[out] reference of y coordinate
*/
void gaussianProjection::forwardProjectStep2(double latInput, double lonInput, double headInput, double &xOutput, double &yOutput)
{
	double dx, dy;

	latInput = DDD2DMS(latInput);
	lonInput = DDD2DMS(lonInput);

	fgConvertBl2xy(latInput, lonInput, CurL0, xOutput, yOutput);
	dx = xOutput - centerX;
	dy = yOutput - centerY;
	yOutput = dx * cos(headInput) + dy * sin(headInput);
	xOutput = dx * -sin(headInput) + dy * cos(headInput);

}

/** \brief backward projection step1
* \param[in] Original of lat coordinate
* \param[in] Original of lon coordinate
*/
void gaussianProjection::backwardProjectStep1(double latOriginal, double lonOriginal)
{
	latOriginal = DDD2DMS(latOriginal);
	lonOriginal = DDD2DMS(lonOriginal);
	fgConvertBl2xy(latOriginal, lonOriginal, CurL0, centerX, centerY);
}

/** \brief backward projection step2
* \param[in]  x coordinate
* \param[in]  y coordinate
* \param[in]  head direction
* \param[out] reference of lat coordinate
* \param[out] reference of lon coordinate
*/
void gaussianProjection::backwardProjectStep2(double xInput, double yInput, double headInput, double & latOutput, double & lonOutput)
{
	double convX, convY;

	convX = xInput * cos(headInput) + yInput*sin(headInput);
	convY = xInput * -sin(headInput) + yInput*cos(headInput);

	fgConvertxy2Bl(convX, convY, CurL0, latOutput, lonOutput);

}

/** \brief set reference lon
* \param[in]  lon coordinate
*/
void gaussianProjection::setReferenceLon(double setLon)
{
	lonReference = setLon;
}

/** \brief set projection parameters
* \param[in]  parameters
*/
void gaussianProjection::setProjectionPara(eEllipCoeffType setPara)
{
	eEllipCoeff = setPara;
}

/** \brief DDD to DMS
* \param[in] DDD
*/
double gaussianProjection::DDD2DMS(double DDD)
{
	double DMS;
	double temp1, temp2;
	temp1 = fmod(DDD, 1);
	DMS = DDD - temp1;
	temp1 = temp1 * 60;
	temp2 = fmod(temp1, 1);
	DMS = DMS + (temp1 - temp2) / 100.0;
	temp2 = temp2 * 60;
	DMS = DMS + (temp2) / 10000.0;
	return DMS;
}

/** \brief DMS to RAD
* \param[in] DMS
*/
double gaussianProjection::convertDms2Rad(double Dms)
{
	double Degree;
	double Miniute;
	double Second;
	double Rad;
	int    Sign;

	if (Dms >= 0)
	{
		Sign = 1;
	}
	else
	{
		Sign = -1;
	}
	Dms = fabs(Dms);
	Degree = floor(Dms);
	Miniute = floor(fmod(Dms * 100.0, 100.0));
	Second = fmod(Dms * 10000.0, 100.0);
	Rad = Sign * (Degree + Miniute / 60.0 + Second / 3600.0) * gp::pi / 180.0;

	return Rad;
}

/** \brief Lat,Lon to x,y
* \param[in]  Lat
* \param[in]  Lon
* \param[in]  Lon Reference
* \param[out] reference of original x coordinate
* \param[out] reference of original y coordinate
*/
void gaussianProjection::fgConvertBl2xy(double ConvB, double Convl, double CurL0, double &xOriginal, double &yOriginal)
{
	double X, N, t, t2, p, p2, eta2;
	double sinB, cosB;
	double B;
	double l;

	//save current B&l
	B = convertDms2Rad(ConvB);
	l = convertDms2Rad(Convl);

	X = eEllipCoeff.A1 * B * 180.0 / gp::pi + eEllipCoeff.A2 * sin(2 * B) +
		eEllipCoeff.A3 * sin(4 * B) + eEllipCoeff.A4 * sin(6 * B) +
		eEllipCoeff.A5 * sin(8 * B);
	sinB = sin(B);
	cosB = cos(B);
	t = tan(B);
	t2 = t * t;
	N = eEllipCoeff.a / sqrt(1 - eEllipCoeff.e2 * sinB * sinB);
	p = cosB * (l - CurL0);
	p2 = p * p;
	eta2 = cosB * cosB * eEllipCoeff.e2 / (1 - eEllipCoeff.e2);
	xOriginal = X + N * t * ((0.5 + ((5 - t2 + 9 * eta2 + 4 * eta2 * eta2) / 24.0 +
		(61 - 58 * t2 + t2 * t2) * p2 / 720.0) * p2) * p2);
	yOriginal = N * p * (1 + p2 * ((1 - t2 + eta2) / 6.0 + p2 * (5 - 18 * t2 + t2 * t2
		+ 14 * eta2 - 58 * eta2 * t2) / 120.0));
	yOriginal += 500000;
}

/** \brief x,y to Lat,Lon
* \param[in]  x
* \param[in]  y
* \param[in]  Lon Reference
* \param[out] reference of lat coordinate
* \param[out] reference of lon coordinate
*/
void gaussianProjection::fgConvertxy2Bl(double Convx, double Convy, double CurL0, double & latOutput, double & lonOutput)
{
	double sinB, cosB, t, t2, N, eta2, V, yN;
	double preB0, B0;
	double deta;
	double B;
	double l;
	double x, y;

	//save current x&y
	x = Convy + centerX;
	y = Convx + centerY;

	y -= 500000;
	B0 = x / eEllipCoeff.A1;

	do
	{
		preB0 = B0;
		B0 = B0 * gp::pi / 180.0;
		B0 = (x - (eEllipCoeff.A2 * sin(2 * B0) + eEllipCoeff.A3 * sin(4 * B0) +
			eEllipCoeff.A4 * sin(6 * B0) + eEllipCoeff.A5 * sin(8 * B0))) /
			eEllipCoeff.A1;
		deta = fabs(B0 - preB0);
	} while (deta > 0.000000001);

	B0 = B0 * gp::pi / 180.0;
	//B = ConvertRad2Dms(B0);
	sinB = sin(B0);
	cosB = cos(B0);
	t = tan(B0);
	t2 = t * t;
	N = eEllipCoeff.a / sqrt(1 - eEllipCoeff.e2 * sinB * sinB);
	eta2 = cosB * cosB * eEllipCoeff.e2 / (1 - eEllipCoeff.e2);
	V = sqrt(1 + eta2);
	yN = y / N;
	B = B0 - (yN * yN - (5 + 3 * t2 + eta2 - 9 * eta2 * t2) * yN * yN * yN * yN /
		12.0 + (61 + 90 * t2 + 45 * t2 * t2) * yN * yN * yN * yN * yN * yN / 360.0)
		* V * V * t / 2;
	l = CurL0 + (yN - (1 + 2 * t2 + eta2) * yN * yN * yN / 6.0 + (5 + 28 * t2 + 24
		* t2 * t2 + 6 * eta2 + 8 * eta2 * t2) * yN * yN * yN * yN * yN / 120.0) / cosB;

	latOutput = B / gp::pi * 180;
	lonOutput = l / gp::pi * 180;
}
