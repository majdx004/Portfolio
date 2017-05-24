#include "TRIAD.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>
#include <cmath>

using namespace matrix;

// declare Matrix<float, 4, 1> TRIAD(Matrix<float, 3, 1> s_B, Matrix<float, 3, 1> m_B, Matrix<float, 3, 1> s_i, Matrix<float, 3, 1> m_i);

Matrix<float, 4, 1> TRIAD(Matrix<float, 3, 1> s_B, Matrix<float, 3, 1> m_B, Matrix<float, 3, 1> s_i, Matrix<float, 3, 1> m_i) {

	/// compute second base vector
	Matrix<float, 3, 3> S = skew(m_B);
	Vector3f t2bu = S * s_B;
	Vector3f t2b = t2bu / norm(t2bu(0), t2bu(1), t2bu(2));
		
	Matrix<float, 3, 3> S1 = skew(m_i);
	Vector3f t2iu = S1 * s_i;
	Vector3f t2i = t2iu / 
		norm(t2iu(0), t2iu(1), t2iu(2));
		
	// computer third base vector to complete triad
	Matrix<float, 3, 3> S2 = skew(m_B);
	Vector3f t3b = S2 * t2b;
	Matrix<float, 3, 3> S3 = skew(m_i);
	Vector3f t3i = S3 * t2i;
	
	// construct 2 rotation matrices
	
	Matrix<float, 3, 3> A;
	A(0, 0) = m_B(0, 0);
	A(1, 0) = m_B(1, 0);
	A(2, 0) = m_B(2, 0);
	A(0, 1) = t2b(0);
	A(1, 1) = t2b(1);
	A(2, 1) = t2b(2);
	A(0, 2) = t3b(0);
	A(1, 2) = t3b(1);
	A(2, 2) = t3b(2);

	Matrix<float, 3, 3> B;
	B(0, 0) = m_i(0, 0);
	B(1, 0) = m_i(1, 0);
	B(2, 0) = m_i(2, 0);
	B(0, 1) = t2i(0);
	B(1, 1) = t2i(1);
	B(2, 1) = t2i(2);
	B(0, 2) = t3i(0);
	B(1, 2) = t3i(1);
	B(2, 2) = t3i(2);
	
	// compute rotaion matrix
	Matrix<float, 3, 3> B_T = B.transpose();
	Matrix3f R_i_B = A * B_T;
	
	//convert to quaternion
	
	Matrix<float, 4, 1> q_init;
	q_init = dcm2quat(R_i_B);
	if (q_init(0, 0) < 0)
		q_init = -q_init;
	
	return(q_init, R_i_B);	

}

// declare float norm (float x, float y, float z);

float norm (float x, float y, float z) {

	float n = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	return(n);

}

// declare Matrix<float, 4, 1> dcm2quat(Matrix<float, 3, 3> dcm);

Matrix<float, 4, 1> dcm2quat(Matrix<float, 3, 3> dcm) {

	float q4, q3, q2, q1;
	
	float trace = dcm(0, 0) + dcm(1, 1) + dcm(2, 2);
	q4 = 0.5 * sqrt(1 + trace);
	
	q3 = (dcm(0, 1) - dcm(1, 0)) / (4*q4);
	
	q2 = (dcm(2, 0) - dcm(0, 2)) / (4*q4);
	
	q1 = (dcm(1, 2) - dcm(2, 1)) / (4*q4);
	
	float quat_vals[4] = {q1, q2, q3, q4};
	Matrix<float, 4, 1> quat(quat_vals);
	
	return(quat);

}

// declare Matrix<float, 3, 3> skew(Matrix<float, 3, 1> a);

Matrix<float, 3, 3> skew(Matrix<float, 3, 1> a) {

	float S_data[9] = {0, -a(2, 0), a(1, 0), a(2, 0), 0, -a(0, 0), -a(1, 0), a(0, 0), 0};
	Matrix3f S(S_data);
		
	return S;
}