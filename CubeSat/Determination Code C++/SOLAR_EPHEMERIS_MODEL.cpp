#include "SOLAR_EPHEMERIS_MODEL.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>
#include <cmath>

using namespace matrix;

// declare float SOLAR_EPHEMERIS_MODEL(Matrix<float, 3, 1> UTC, float lat, float lon, float alt_m);

float SOLAR_EPHEMERIS_MODEL(Matrix<float, 3, 1> UTC, float lat, float lon, float alt_m) {
	// Julian Date
	float T_JD, I, J, K;
	I = UTC(0); // Year
	J = UTC(1); // Month
	K = UTC(2); // Day
	T_JD = K - 32075 + 1461 * (I + 4800 + (J - 14) / 12) / 4 + 367 * (J - 2 - (J - 14) / 12 * 12) * 2 / 12 - 3 * ((I + 4900 + (J - 14) / 12) / 100) / 4;

	// Planetary Ephemeris
	// Need to find a solar ephemeris model
	// output: X_ICRF2_km

	// lla2ecef
	float  a = 6378137.0;              //WGS-84 semi-major axis
    float e2 = .0066943799901377997;  //WGS-84 first eccentricity squared
	float n;
	Matrix<float, 3, 1> ecef;
	n = a/sqrt( 1 - e2*sin( lat )*sin( lat ) );
    ecef(0, 0) = ( n + alt )*cos( lat )*cos( lon );    //ECEF x
    ecef(1, 0) = ( n + alt )*cos( lat )*sin( lon );    //ECEF y
    ecef(2, 0) = ( n*(1 - e2 ) + alt )*sin( lat );          //ECEF z

	// ecef2eci
	// Need to find code for ecef to eci
	// output: pos_eci_m

    Pos_eci_km = pos_eci_m * .001;

    // s_ECI: Summation and Normalization
	Matrix<float, 3, 1> s_ECI_temp;
	Matrix<float, 3, 1> s_ECI;
    s_ECI_temp = X_ICRF2_km - Pos_eci_km;
    s_ECI = s_ECI_temp/norm(s_ECI_temp(0,0), s_ECI_temp(1,0), s_ECI_temp(2,0));

	return(s_ECI);
}

// declare float norm (float x, float y, float z);

float norm (float x, float y, float z) {

	float n = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	return(n);

}