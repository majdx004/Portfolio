#ifndef EKF_H_
#define EKF_H_

using namespace matrix;

float EKF(Matrix<float, 3, 1> W, Matrix<float, 3, 1> s_B, Matrix<float, 3, 1> m_B, Matrix<float, 3, 1> s_i, Matrix<float, 3, 1> m_i, Matrix<float, 4, 1> q_prev, Matrix<float, 3, 3> P_prev);

float norm (float x, float y, float z);

float disrw(Matrix<float, 3, 3> F, Matrix<float, 3, 3> G, float T, Matrix<float, 3, 3> Rwpsd);

float quatnormalize(Matrix<float, 4, 1> q);

#endif