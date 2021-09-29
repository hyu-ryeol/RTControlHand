/*
 * Kinematics.h
 *
 *  Created on: 2018. 7. 25.
 *      Author: Administrator
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;

typedef struct{
	float x;
	float y;
	float z;
}Cartesian;

typedef struct{
	float q[5];
	float q_d[5];
}Joint;

#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3

class Kinematics {
public:
	Kinematics();
	virtual ~Kinematics();

	Joint InverseKinematics(float xd_mm, float yd_mm);

protected:
	Matrix4f TransMAT(int axis, float mm);
	Matrix4f RotMAT(int axis, float rad);

	Cartesian c_mm;
	Joint th_rad;
	const float k_pole_mm = 120.0; //mm
	const float k_radius_mm = 60.0; //mm
private:
	Matrix4f P[5];
	Matrix4f T_l;
	Matrix4f T[5];
};

#endif /* KINEMATICS_H_ */
