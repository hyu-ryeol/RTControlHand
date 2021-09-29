/*
 * Controller.h
 *
 *  Created on: 2018. 7. 27.
 *      Author: Administrator
 */

#ifndef CONTROL_CONTROLLER_H_
#define CONTROL_CONTROLLER_H_

#include "Kinematics.h"
#include "../main.h"




class Controller: public Kinematics {
public:
	Controller();
	virtual ~Controller();

public:
	 void PDcontroller(struct StateInfo *joint, struct MotorInfo *motor, char key);
	 void EncToPos(struct StateInfo *joint, struct MotorInfo *motor);
	 void Gravity(void);
private:

	float dt, c_input[NUM_CLIENT];
	float aq[NUM_CLIENT];
	float aq_p[NUM_CLIENT];
	float aq_d[NUM_CLIENT];

	float q_err[NUM_CLIENT];
	float q_err_int[NUM_CLIENT];

	float cur[NUM_CLIENT];


	float kp[NUM_CLIENT], kd[NUM_CLIENT], ki[NUM_CLIENT];
	float gravity;
	const float gravity_const = 9.8;
	const float link_mass = 0.5; //kg
	const float link_length = 0.15; //m


};

#endif /* CONTROL_CONTROLLER_H_ */
