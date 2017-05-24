#ifndef TRIAD_H_
#define TRIAD_H_

using namespace matrix;

Matrix<float, 4, 1> TRIAD(Matrix<float, 3, 1> s_B, Matrix<float, 3, 1> m_B, Matrix<float, 3, 1> s_i, Matrix<float, 3, 1> m_i);

float norm (float x, float y, float z);

Matrix<float, 3, 3> skew(Matrix<float, 3, 1> a);

Matrix<float, 4, 1> dcm2quat(Matrix<float, 3, 3> dcm);

#endif
