/*
 * IMU.h
 *
 *  Created on: Mar 11, 2015
 *      Author: myavari
 */

#ifndef SRC_APPLICATION_IMU_H_
#define SRC_APPLICATION_IMU_H_

#include "port.h"
#include "compiler.h"

typedef struct
{
	double x;
	double y;
	double z;
} vector;

typedef struct {
	double a; 	//magnitude
	double b;	//Vec.x
	double c;	//Vec.y
	double d;	//Vec.z
} quaternion;



void updateAccel(vector accelVec_b);
void updateGyro(vector gyroVec);
vector getAcceleration();
vector getPosition();
vector getVelocity();
vector getOrientation();

#endif /* SRC_APPLICATION_IMU_H_ */
