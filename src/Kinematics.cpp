#include "Kinematics.h"


Kinematic::Kinematic()
	: l1(55), l2(13), l3(110), l4(120),
	L(185), W(80), H(180)
{
	for (int i = 0; i < 12; i++)
		thetas[i / 3][i % 3] = 0;
}

void Kinematic::bodyIK(float returns[4][4][4], float angles[3], float center[3]) {

	if (center[0] > 110) center[0] = 110;
	if (center[0] < -270) center[0] = -270;
	if (center[1] < -60) center[1] = -60;
	if (center[1] > 95) center[1] = 95;
	if (center[2] < -110) center[2] = -110;
	if (center[2] > 110) center[2] = 110;

	if (angles[0] > 0.84) angles[0] = 0.84;
	if (angles[0] < -0.84) angles[0] = -0.84;
	if (angles[1] > 1.10) angles[1] = 1.10;
	if (angles[1] < -1.10) angles[1] = -1.10;
	if (angles[2] > 0.84) angles[2] = 0.84;
	if (angles[2] < -0.68) angles[2] = -0.68;

	float Rx[4][4] = { {1,0,0,0},{0,cos(angles[0]),-sin(angles[0]),0},{0,sin(angles[0]),cos(angles[0]),0},{0,0,0,1} };
	float Ry[4][4] = { {cos(angles[1]), 0, sin(angles[1]), 0}, {0,1,0,0}, {-sin(angles[1]), 0, cos(angles[1]), 0}, {0,0,0,1} };
	float Rz[4][4] = { {cos(angles[2]), -sin(angles[2]), 0, 0}, {sin(angles[2]), cos(angles[2]),0,0}, {0,0,1,0}, {0,0,0,1} };
	float T[4][4] = { {0,0,0,center[0]}, {0,0,0,center[1]}, {0,0,0,center[2]}, {0,0,0,0} };

	float Rxyz[4][4] = { 0, };
	float Ryz[4][4] = { 0, };

	mat.dot(Ryz, Ry, Rz);
	mat.dot(Rxyz, Rx, Ryz);

	float Tm[4][4] = { 0, };
	mat.add(Tm, Rxyz, T);

	mat.dot(returns[0], Tm, func1);
	mat.dot(returns[1], Tm, func2);
	mat.dot(returns[2], Tm, func3);
	mat.dot(returns[3], Tm, func4);
}

void Kinematic::legIK(float returns[3], float point[4]) /* 3 float, 3 float */

{
	float x = point[0]; // -
	float y = point[1]; // |
	float z = point[2]; // /

	float F = pow(x,2) + pow(y,2) - pow(l1,2);
	
	if (F < 0) {
		F = l1;
		Serial.print(" F error");
	}
	else F = sqrt(F);

	float G = F - l2;
	float H = sqrt(pow(G,2) + pow(z,2));
	returns[0] = - atan2(y, x) - atan2(F, -l1);

	float D = (pow(H, 2) - pow(l3, 2) - pow(l4, 2)) / (2 * l3 * l4);

	if (D > 1 || D < -1) {
		returns[2] = 0;
		Serial.print(" acos error");
	}
	else returns[2] = acos(D);

	returns[1] = atan2(z, G) - atan2(l4 * sin(returns[2]), l3 + l4 * cos(returns[2]));
}

//void Kinematic::calcLegPoints(float* returns, float* angles) {
//	int a = 0;
//	a = 10;
//}

void Kinematic::calcIK(float returns[4][3], float Lp[4][4], float angles[3], float center[3]) {
	
	float T[4][4][4] = { 0, };
	bodyIK(T, angles, center);

	{
		float invT[4][4] = { 0, };
		mat.inv(invT, T[0]);
		float dotT[4] = { 0, };
		Lp[0][3] = 1;
		mat.dot(dotT, invT, Lp[0]);
		legIK(returns[0], dotT);
	}
	{
		float invT[4][4] = { 0, };
		mat.inv(invT, T[1]);
		float dotT[4] = { 0, };
		float dotIT[4] = { 0, };
		Lp[1][3] = 1;
		mat.dot(dotT, invT, Lp[1]);
		mat.dot(dotIT, Ix, dotT);
		legIK(returns[1], dotIT);
	}
	{
		float invT[4][4] = { 0, };
		mat.inv(invT, T[2]);
		float dotT[4] = { 0, };
		Lp[2][3] = 1;
		mat.dot(dotT, invT, Lp[2]);
		legIK(returns[2], dotT);
	}
	{
		float invT[4][4] = { 0, };
		mat.inv(invT, T[3]);
		float dotT[4] = { 0, };
		float dotIT[4] = { 0, };
		Lp[3][3] = 1;
		mat.dot(dotT, invT, Lp[3]);
		mat.dot(dotIT, Ix, dotT);
		legIK(returns[3], dotIT);
	}
}

//void Kinematic::legFK(float returns[3], float point[4])  /* 3 float, 3 float */
//{
//	float theta1 = angles[0] * pi / 180;
//	float theta2 = angles[2] * pi / 180;
//	float theta3 = angles[3] * pi / 180;
//
//	returns[2] = (float)(l3 * sin(theta2 - pi / 2) + l4 * sin(theta2 - theta3));
//
//	float yp = (float)(l3 * cos(theta2 - pi / 2) + l4 * cos(theta2 - theta3));
//
//	float alpha = atan2(yp + l2, l1);
//
//	float beta = alpha - theta1;
//
//	returns[1] = sqrt(pow(yp + l2, 2) + pow(l1, 2)) * sin(beta);
//
//	returns[0] = (float)(returns[1] / tan(beta));
//}
//void Kinematic::testlegIK(float* returns, float* point);