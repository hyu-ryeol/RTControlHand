/*
 * Controller.cpp
 *
 *  Created on: 2018. 7. 27.
 *      Author: Administrator
 */

#include "Controller.h"

Controller::Controller()
:gravity(0),
 dt(0)
{
	for(int i=0; i<NUM_CLIENT; ++i)
	{
		c_input[i] = 0;
		aq[i]=0;
		aq_p[i] = 0;
		aq_d[i] = 0;

		cur[i] = 0;

		q_err[i]=0;
		q_err_int[i] = 0;

		kp[i] = 1.0;
		kd[i] = 0.10;
		ki[i] = 10.0;
	}

}

Controller::~Controller() {

}

void Controller::EncToPos(StateInfo *joint, MotorInfo *motor)
{

	for(int i=0; i<NUM_CLIENT; ++i)
	{
		if(i == 5)
		{

			aq[i] = ((float)(joint->enc_pos[i]))/(250.0*motor->gear_ratio[i])*2.0*M_PI;
			joint->aq[i] = aq[i];
			joint->atoq[i] = motor->rate_torque[i]/1000.0*(float)joint->toq[i]*motor->gear_ratio[i];
		}
		else
		{
			aq[i] = ((float)(joint->enc_pos[i]))/(4.0*1024.0*motor->gear_ratio[i])*2.0*M_PI;
			joint->aq[i] = aq[i];
			joint->atoq[i] = motor->rate_torque[i]/1000.0*(float)joint->toq[i]*motor->gear_ratio[i];
		}
	}


}

void Controller::PDcontroller(StateInfo *joint, MotorInfo *motor, char key)
{
	dt = joint->dt;
	gravity = link_mass*link_length*gravity_const*sinf(aq[3]+M_PI/4);

	for(int i=0; i<NUM_CLIENT; ++i)
	{
		joint->aq[i] = (float)joint->enc_pos[i];
		q_err[i] = (joint->dq[i] - joint->aq[i]);

		q_err_int[i] += q_err[i]*dt;
		aq_d[i] = (aq[i] - aq_p[i])/dt;

		if(key == 49) //
		{
			if(i == 1 || i==3)
			{
				joint->int16_val[i] = -200;
			}
			else if(i==2 || i==4)
			{
				joint->int16_val[i] = -200;
			}
			else
			{
				joint->int16_val[i] = 0;
			}
		}
		else if(key == 50)
		{
			if(i == 1 || i==3)
			{
				joint->int16_val[i] = 200;
			}
			else if(i==2 || i==4)
			{
				joint->int16_val[i] = 200;
			}
			else
			{
				joint->int16_val[i] = 0;
			}
		}
		else if(key == 51)
		{
			if(i == 6 || i==8)
			{
				joint->int16_val[i] = -80;
			}
			else if(i==7 || i==9)
			{
				joint->int16_val[i] = -300;
			}
			else
			{
				joint->int16_val[i] = 0;
			}
		}
		else if(key == 52)
		{
			if(i == 1 || i==3)
			{
				joint->int16_val[i] = -200;
			}
			else if(i==2 || i==4)
			{
				joint->int16_val[i] = -200;
			}
			else if(i == 6 || i==8)
			{
				joint->int16_val[i] = -300;
			}
			else if(i==7 || i==9)
			{
				joint->int16_val[i] = -300;
			}
			else
			{
				joint->int16_val[0] = 0;
				joint->int16_val[5] = -100;
			}
		}
		else if(key == 53)
		{
			if(i == 1 || i==3)
			{
				joint->int16_val[i] = -200;
			}
			else if(i==2 || i==4)
			{
				joint->int16_val[i] = -200;
			}
			else if(i == 6 || i==8)
			{
				joint->int16_val[i] = 200;
			}
			else if(i==7 || i==9)
			{
				joint->int16_val[i] = 200;
			}
			else
			{
				joint->int16_val[0] = 0;
				joint->int16_val[5] = -100;
			}
		}
		else if(key == 54)
		{
			if(i == 1 || i==3)
			{
				joint->int16_val[i] = 200;
			}
			else if(i==2 || i==4)
			{
				joint->int16_val[i] = 200;
			}
			else if(i == 6 || i==8)
			{
				joint->int16_val[i] = -100;
			}
			else if(i==7 || i==9)
			{
				joint->int16_val[i] = -300;
			}
			else
			{
				joint->int16_val[0] = 0;
				joint->int16_val[5] = -100;
			}
		}
		else
		{
			joint->int16_val[i] = 0;
		}
		aq_p[i] = aq[i];
	}
}
