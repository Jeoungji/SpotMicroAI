#pragma once
#include "Arduino.h"
#include "Matrix.h"

#define pi 3.141592653589793238462643383
#define sHp 1
#define cHp 0

class Kinematic {
private:
	int l1;
	int l2;
	int l3;
	int l4;

	float L;
	float W;
	float H;

	float thetas[4][3];
	Matrix mat;

	float func1[4][4] = { {cHp, 0, sHp, L / 2}, {0,1,0,0},{-sHp,0,cHp,W / 2}, {0,0,0,1} };
	float func2[4][4] = { {cHp, 0, sHp, L / 2}, {0,1,0,0},{-sHp,0,cHp,- W / 2}, {0,0,0,1} };
	float func3[4][4] = { {cHp, 0, sHp, - L / 2}, {0,1,0,0},{-sHp,0,cHp,W / 2}, {0,0,0,1} };
	float func4[4][4] = { {cHp, 0, sHp, - L / 2}, {0,1,0,0},{-sHp,0,cHp,- W / 2}, {0,0,0,1} };
	float Ix[4][4] = { {-1,0,0,0},{0,1,0,0}, {0,0,1,0}, {0,0,0,1} };
public:
	Kinematic();
	void bodyIK(float returns[4][4][4], float angles[3], float center[3]);
	void legIK(float returns[3], float point[4]);
	//void calcLegPoints(float* returns, float* angles);
	void calcIK(float returns[4][3], float Lp[4][4], float angles[3], float center[3]);
	//void legFK(float returns[3], float point[4]);

	//void testlegIK(float* returns, float* point);
public:
	//void setL_m(int num) { L_m = L_m + num; }
	//void setM_m(int num) { M_m = M_m + num; }
	//int getL() { return L; }
	//int getM() { return M; }
	//void addL(int num) { setL(getL() + num); }
	//void addM(int num) { setM(getM() + num); }
};