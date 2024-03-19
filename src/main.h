#pragma once

#ifndef _MAIN
#define _MAIN

#include <SPI.h>
#include <MPU9250.h>
#include <Servo12.h>
#include "Controllers.h"
#include "Kinematics.h"
#include "Matrix.h"
#include <Filter.h>
#include <SpotMicro.h>

// All Debug Control
#define DEBUGING        1

#define MPU9250DEBUG    1

#if DEBUGING == 0
#undef  MPU9250DEBUG
#define MPU9250DEBUG 0

#endif

#define VSENSOR 41
#define MPOWER  29
#define INTERVAL_MS     10
#define pi 3.141592653589793238462643383

typedef struct Bfloat {
    bool ok;
    float data;
} Bfloat;

typedef struct Bint {
    bool ok;
    int data;
} Bint;

typedef struct XYZ {
    float x;
    float y;
    float z;
} XYZ;



typedef struct XYZ_c {
  int8_t x;
  int8_t y;
  int8_t z;
} XYZ_c;

#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  

#endif