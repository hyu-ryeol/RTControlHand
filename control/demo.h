/*
 * demo.h
 *
 *  Created on: 2018. 7. 19.
 *      Author: Administrator
 */

#ifndef DEMO_H_
#define DEMO_H_


#include "../main.h"
#include "../can_define.h"
#include "Kinematics.h"

class Demo{

public:
	Demo();
	virtual ~Demo();

public:


private:
	int type_p;
	int step;
};

#endif /* DEMO_H_ */
