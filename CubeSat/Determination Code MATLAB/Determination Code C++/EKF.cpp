#include "EKF.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>
#include <cmath>

using namespace matrix;

// declare float EKF(Matrix<float, 3, 1> W, Matrix<float, 3, 1> s_B, Matrix<float, 3, 1> m_B, Matrix<float, 3, 1> s_i, Matrix<float, 3, 1> m_i, Matrix<float, 4, 1> q_prev, Matrix<float, 3, 3> P_prev);

float EKF(Matrix<float, 3, 1> W, Matrix<float, 3, 1> s_B, Matrix<float, 3, 1> m_B, Matrix<float, 3, 1> s_i, Matrix<float, 3, 1> m_i, Matrix<float, 4, 1> q_prev, Matrix<float, 3, 3> P_prev) {

	float d2r;
	float r2d;
	float dt;
	d2r = 3.141529/180;
	r2d = 1/d2r;
	dt = .1;    // Period for data processing

	// CubeSat ADCS - Gyro Integration
	float B_data[16] = {0, -W(0), -W(1), -W(2), W(0), 0, W(2), -W(1), W(1), -W(2), 0, W(0), W(2), W(1), -W(0), 0};
	Matrix<float, 4, 4> B(B_data);
	Matrix<float, 1, 4> dq;
	dq = .5*(B*q_prev).transpose();
	Matrix<float, 1, 4> qgyro;
	qgyro = q_prev.transpose() + dt * dq;
	float ones_data[4] = {1, 1, 1, 1};
	Matrix<float, 1, 4> ones(ones_data);
	qgyro = qgyro/(quatnormalize(qgyro)*ones);

	// Propagate State Covariance
	Matrix<float, 3, 3> I;
	I.setIdentity();
	G = -.5 * I;
	Matrix<float, 3, 3> Rw;
	Rw = I * pow(.005 * d2r, 2);
	float F_data[9] = {0, W(2), -W(1), -W(2), 0, W(0), W(1), -W(0), 0};
	Matrix<float, 3, 3> F(F_data);
	Matrix<float, 3, 3> PHI = I + F * .1;
	Matrix<float, 3, 3> Cd = disrw(F, G, .1, Rw);
	Matrix<float, 3, 3> P = PHI * P_prev * PHI.tranpose() + Cd;

	// construct z, H, Rz
	float zero_data[3] = {0, 0, 0};
	Matrix<float, 1, 3> m_b_est(zero_data);
	Matrix<float, 1, 3> s_b_est(zero_data);
	Matrix<float, 3, 3> C_n2b;
	C_n2b.setIdentity();
	m_b_est = qgyro.rotate(m_i.tranpose()); // need to check 
	s_b_est = qgyro.rotate(s_i.transpose()); // need to check
	float z_data[6] = {m_B(0, 0) - m_b_est(0, 0), m_B(1, 0) - m_b_est(0, 1), m_B(2, 0) - m_b_est(0, 2), s_B(0, 0) - s_b_est(0, 0), s_B(1, 0) - s_b_est(0, 1), s_B(2, 0) - s_b_est(0, 2)};
	Matrix<float, 6, 1> z(z_data);
	float H_data[18] = {0, m_B(2), -m_B(1), -m_B(2), 0, m_B(0), m_B(1), -m_B(0), 0, 0, s_B(2), -s_B(1), -s_B(2), 0, s_B(0), s_B(1), -s_B(0), 0};
	Matrix<float, 6, 3> H(H_data);
	C_n2b = dcm1(q_prev.tranpose());
	Matrix<float, 3, 3> R_m;
	Matrix<float, 3, 3> R_s;
	R_m = I * .02;
	R_s = I * .05;
	float zeroes_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	Matrix<float, 3, 3> zeroes(zeroes_data);
	Matrix<float, 3, 3> pone = C_n2b*R_m*C_n2b/transpose();
	Matrix<float, 3, 3> pthree = C_n2b*R_s*C_n2b.tranpose();
	Matrix<float, 6, 6> R_z;
	Rz(0, 0) = pone(0, 0);
	Rz(0, 1) = pone(0, 1);
	Rz(0, 2) = pone(0, 2);
	Rz(1, 0) = pone(1, 0);
	Rz(1, 1) = pone(1, 1);
	Rz(1, 2) = pone(1, 2);
	Rz(2, 0) = pone(2, 0);
	Rz(2, 1) = pone(2, 1);
	Rz(2, 2) = pone(2, 2);

	Rz(0, 3) = zeroes(0, 0);
	Rz(0, 4) = zeroes(0, 1);
	Rz(0, 5) = zeroes(0, 2);
	Rz(1, 3) = zeroes(1, 0);
	Rz(1, 4) = zeroes(1, 1);
	Rz(1, 5) = zeroes(1, 2);
	Rz(2, 3) = zeroes(2, 0);
	Rz(2, 4) = zeroes(2, 1);
	Rz(2, 5) = zeroes(2, 2);

	Rz(3, 0) = zeroes(0, 0);
	Rz(3, 1) = zeroes(0, 1);
	Rz(3, 2) = zeroes(0, 2);
	Rz(4, 0) = zeroes(1, 0);
	Rz(4, 1) = zeroes(1, 1);
	Rz(4, 2) = zeroes(1, 2);
	Rz(5, 0) = zeroes(2, 0);
	Rz(5, 1) = zeroes(2, 1);
	Rz(5, 2) = zeroes(2, 2);

	Rz(3, 3) = pthree(0, 0);
	Rz(3, 4) = pthree(0, 1);
	Rz(3, 5) = pthree(0, 2);
	Rz(4, 3) = pthree(1, 0);
	Rz(4, 4) = pthree(1, 1);
	Rz(4, 5) = pthree(1, 2);
	Rz(5, 3) = pthree(2, 0);
	Rz(5, 4) = pthree(2, 1);
	Rz(5, 5) = pthree(2, 2);

	// Compute Kalman Gain and update q
	Matrix<float, 3, 6> L;
	L = P * H.transpose() * pow(H*P*H.transpose() + Rz, -1);
	Matrix<float, 3, 1> q_e;
	q_e = L*z;
	float qe_data[9] = {1, q_e(0, 0), q_e(1, 0), q_e(2, 0)};
	Matrix<float, 1, 4> qe(qe_data);
	Matrix<float, 1, 4> q;
	q = qgyro * qe;
	q.normalize();

	if q(0) < 0
		q = -q;
	end

	// Update state error covariance
	P = (I - L*H) * P;
	return(q, P, qe);
}

// declare float norm (float x, float y, float z);

float norm (float x, float y, float z) {

	float n = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	return(n);

}

// declare float disrw(Matrix<float, 3, 3> F, Matrix<float, 3, 3> G, float T, Matrix<float, 3, 3> Rwpsd)

float disrw(Matrix<float, 3, 3> F, Matrix<float, 3, 3> G, float T, Matrix<float, 3, 3> Rwpsd) {

	Matrix<float, 3, 3> I;
	I.setIdentity();
	Matrix<float, 3, 3> Q_k = (I + T*F)*(T*G*Rwpsd*G.transpose());
	return(Q_k);
}

// declare float quatnormalize(Matrix<float, 4, 1> q);

float quatnormalize(Matrix<float, 4, 1> q) {

	float qmag = sqrt(pow(q(0,0),2) + pow(q(1,0),2) + pow(q(2,0),2) + pow(q(3,0),2));
	q = q / qmag;
	return(q)
}


