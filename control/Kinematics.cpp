/*
 * Kinematics.cpp
 *
 *  Created on: 2018. 7. 25.
 *      Author: Administrator
 */

#include "Kinematics.h"

Kinematics::Kinematics() {


}

Kinematics::~Kinematics() {

}

Joint Kinematics::InverseKinematics(float xd_mm, float yd_mm)
{
	float *th_tmp = new float[2];
	float *w = new float[4];

	float zd_mm = sqrtf(k_pole_mm*k_pole_mm - xd_mm*xd_mm - yd_mm*yd_mm);
	th_tmp[0] = -atan2f(xd_mm, yd_mm);
	th_tmp[1] = atan2f(sqrtf(xd_mm*xd_mm + yd_mm*yd_mm), zd_mm);

	T[0] = RotMAT(Z_AXIS, th_tmp[0]+M_PI);
	T[1] = RotMAT(X_AXIS, M_PI/2)*RotMAT(Z_AXIS, th_tmp[1]);
	T[2] = RotMAT(X_AXIS, -M_PI/2)*TransMAT(Z_AXIS, k_pole_mm);
	T[3] = RotMAT(X_AXIS, M_PI/2)*RotMAT(Z_AXIS, th_tmp[1]);
	T[4] = RotMAT(X_AXIS, -M_PI/2)*RotMAT(Z_AXIS, -th_tmp[0]-M_PI);

	T_l = T[0]*T[1]*T[2]*T[3]*T[4];

	P[0] = TransMAT(X_AXIS, k_radius_mm) - T_l*TransMAT(X_AXIS, k_radius_mm);
	P[1] = TransMAT(X_AXIS, -k_radius_mm) - T_l*TransMAT(X_AXIS, -k_radius_mm);
	P[2] = TransMAT(Y_AXIS, k_radius_mm) - T_l*TransMAT(Y_AXIS, k_radius_mm);
	P[2] = TransMAT(Y_AXIS, -k_radius_mm) - T_l*TransMAT(Y_AXIS, -k_radius_mm);

	for(int i=0; i<4; ++i)
		w[i] = sqrtf(P[i](0,3)*P[i](0,3) + P[i](1,3)*P[i](1,3) + P[i](2,3)*P[i](2,3));

	th_rad.q[0] = (120.0 - w[0])*4/(2*15*M_PI)*2*21*4096;
	th_rad.q[1] = (120.0 - w[2])*4/(2*15*M_PI)*2*21*4096;

	delete[] th_tmp;
	delete[] w;
	return th_rad;
}


Matrix4f Kinematics::RotMAT(int axis, float rad)
{
	Matrix4f ht;
	switch(axis)
	{
	case X_AXIS:
		ht << 1, 0, 0, 0,
			0, cosf(rad), -sinf(rad), 0,
			0, sinf(rad), cosf(rad), 0,
			0, 0, 0, 1;
		break;
	case Y_AXIS:
		ht << cosf(rad), 0, sinf(rad), 0,
			0, 1, 0, 0,
			-sinf(rad), 0, cosf(rad), 0,
			0, 0, 0, 1;
		break;
	case Z_AXIS:
		ht << cosf(rad), -sinf(rad), 0, 0,
			sinf(rad), cosf(rad), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;
	default:
		ht << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;

	};
	return ht;
}

Matrix4f Kinematics::TransMAT(int axis, float mm)
{
	Matrix4f ht;
	switch(axis)
	{
	case X_AXIS:
		ht << 1, 0, 0, mm,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;
	case Y_AXIS:
		ht << 1, 0, 0, 0,
			0, 1, 0, mm,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;
	case Z_AXIS:
		ht << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, mm,
			0, 0, 0, 1;
		break;
	default:
		ht << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;

	};
	return ht;
}
