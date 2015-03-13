/*
 * IMU.c
 *
 *  Created on: Mar 11, 2015
 *      Author: myavari
 */
#include "IMU.h"
#include "arm_math.h"

const vector gVec = { 0, 0, -9.81 };
bool gyroFirst = true;
bool accelFirst = true;
arm_matrix_instance_f32 ROT;
float32_t ROT_f32[9];

static quaternion q_bn = { 0.707, 0.707, 0, 0 };
static quaternion q_nb;
static vector a, v, p, o;

quaternion quaternionMultiply(quaternion q1, quaternion q2) {
	quaternion _q;
	_q.a = q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d;
	_q.b = q1.b * q2.a + q1.a * q2.b - q1.d * q2.c + q1.c * q2.d;
	_q.c = q1.c * q2.a + q1.d * q2.b + q1.a * q2.c - q1.b * q2.d;
	_q.d = q1.d * q2.a - q1.c * q2.b + q1.b * q2.c + q1.a * q2.d;
	return _q;
}

double getGyroDuration() {
	static clock_t glastt = 0;
	double dt = 0;
	if (glastt == 0) {
		glastt = clock();
		gyroFirst = true;
	} else {
		gyroFirst = false;
		dt = (clock() - glastt) / (double) CLOCKS_PER_SEC;
		glastt = clock();
	}

	return dt;
}

quaternion get_qnb() {
	return q_nb;
}

quaternion get_qbn() {
	return q_bn;
}

double getAccelDuration() {
	static clock_t alastt = 0;
	double dt = 0;
	if (alastt == 0) {
		alastt = clock();
		accelFirst = true;
	} else {
		accelFirst = false;
		dt = (clock() - alastt) / (double) CLOCKS_PER_SEC;
		alastt = clock();
	}

	return dt;
}

double getVecMagnitude(vector v) {
	return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

vector vectorDecimalMultiply(vector v, double decimal) {
	vector vr;
	vr.x = v.x * decimal;
	vr.y = v.y * decimal;
	vr.z = v.z * decimal;
	return vr;
}

vector vectorAdd(vector v1, vector v2) {
	vector vr;
	vr.x = v1.x + v2.x;
	vr.y = v1.y + v2.y;
	vr.z = v1.z + v2.z;
	return vr;
}

double radianToDegree(double radian) {
	double degree = radian * 57.295;
	return degree;
}

quaternion qFromVec(vector v, double magnitude) {
	quaternion q;
//	double theta = radianToDegree(magnitude / 2);
	double theta = magnitude / 2;
	q.a = cos(theta);
	q.b = v.x / magnitude * sin(theta);
	q.c = v.y / magnitude * sin(theta);
	q.d = v.z / magnitude * sin(theta);
	return q;
}

quaternion getconju(quaternion q) {
	quaternion qr;
	qr.a = q.a;
	qr.b = -q.b;
	qr.c = -q.c;
	qr.d = -q.d;
	return qr;
}

void creatRotMat(quaternion q) {

	ROT_f32[0] = 1 - (2 * pow(q.c, 2)) - (2 * pow(q.d, 2));
	ROT_f32[1] = 2 * (q.b * q.c - q.a * q.d);
	ROT_f32[2] = 2 * (q.b * q.d + q.a * q.c);
	ROT_f32[3] = 2 * (q.b * q.c + q.a * q.d);
	ROT_f32[4] = 1 - (2 * pow(q.b, 2)) - (2 * pow(q.d, 2));
	ROT_f32[5] = 2 * (q.c * q.d + q.a * q.b);
	ROT_f32[6] = 2 * (q.b * q.d - q.a * q.c);
	ROT_f32[7] = 2 * (q.c * q.d - q.a * q.b);
	ROT_f32[8] = 1 - (2 * pow(q.b, 2)) - (2 * pow(q.c, 2));
	arm_mat_init_f32(&ROT, 3, 3, (float32_t *) ROT_f32);
}

vector qRotation(vector v, quaternion q) {
	vector vr;
	float32_t vr_f32[3];
	float32_t v_f32[3];
	arm_matrix_instance_f32 vMat, vrMat;
	v_f32[0] = v.x;
	v_f32[1] = v.y;
	v_f32[2] = v.z;
	arm_mat_init_f32(&vMat, 3, 1, v_f32);
	arm_mat_init_f32(&vrMat, 3, 1, vr_f32);
	creatRotMat(q);
	arm_mat_mult_f32(&ROT, &vMat, &vrMat);
	vr.x = vrMat.pData[0];
	vr.y = vrMat.pData[1];
	vr.z = vrMat.pData[2];
	return vr;
}

quaternion getNorm(quaternion q) {
	quaternion qr;
	double m;
	m = pow(q.a, 2) + pow(q.b, 2) + pow(q.c, 2) + pow(q.d, 2);
	if (m - 1 > 0.1) {
		m = sqrt(m);
		qr.a = q.a / m;
		qr.b = q.b / m;
		qr.c = q.c / m;
		qr.d = q.d / m;
		return qr;
	} else
		return q;
}

void quaternionToEuler() {
	o.x = atan2(2 * q_nb.c * q_nb.d - 2 * q_nb.a * q_nb.b,
			2 * q_nb.a * q_nb.a + 2 * q_nb.d * q_nb.d - 1);
	o.y = asin(2 * q_nb.b * q_nb.c - 2 * q_nb.a * q_nb.c);
	o.z = atan2(2 * q_nb.b * q_nb.c - 2 * q_nb.a * q_nb.d,
			2 * q_nb.a * q_nb.a + 2 * q_nb.b * q_nb.b - 1);
}

vector getOrientation() {
	return o;
}

void updateGyro(vector gyroVec) {
	vector tempVec;
	double m;
	double dt = getGyroDuration();
	if (gyroFirst == true)
		return;
	quaternion q_dif;
	quaternion w_dif;
	tempVec = vectorDecimalMultiply(gyroVec, -dt);
	m = getVecMagnitude(gyroVec);
	q_dif = qFromVec(tempVec, m);
	q_dif = getNorm(q_dif);
	q_bn = quaternionMultiply(q_dif, q_bn);
//////////////////////////////////////////
	q_nb = getconju(q_bn);

}

vector reduceGravity(vector accelVec) {
	vector rVector;
	rVector.x = accelVec.x + gVec.x;
	rVector.y = accelVec.y + gVec.y;
	rVector.z = accelVec.z + gVec.z;
	return rVector;
}

void updateVelocity(double t) {
	v = vectorAdd(v, vectorDecimalMultiply(a, t));
}

void updatePosition(double t) {
	vector tmpv;
	tmpv = vectorAdd(vectorDecimalMultiply(v, t),
			vectorDecimalMultiply(a, pow(t, 2) / 2));
	p = vectorAdd(p, tmpv);
}

vector getVelocity() {
	return v;
}

vector getAcceleration() {
	return a;
}

vector getPosition() {
	return p;
}

//converts accelration measuered in g unit to m/s^2
vector gToMpSS(vector accel_g) {
	vector accel_mpss;
	accel_mpss.x = accel_g.x * 9.81;
	accel_mpss.y = accel_g.y * 9.81;
	accel_mpss.z = accel_g.z * 9.81;
	return accel_mpss;
}

void updateAccel(vector accelVec_b) {
	double dt = getAccelDuration();
	if (accelFirst == true)
		return;
	accelVec_b = gToMpSS(accelVec_b);
	vector accelVec_n = qRotation(accelVec_b, q_bn);
	a = reduceGravity(accelVec_n);
	updatePosition(dt);
	updateVelocity(dt);
	quaternionToEuler();
}

