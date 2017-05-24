#ifndef  SOLAR_EPHEMERIS_MODEL_H_
#define  SOLAR_EPHEMERIS_MODEL_H_

using namespace matrix;

float SOLAR_EPHEMERIS_MODEL(Matrix<float, 3, 1> UTC, float lat, float lon, float alt_m);

float norm (float x, float y, float z);

#endif