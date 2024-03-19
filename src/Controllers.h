#pragma once

#ifndef _CONTROLLERS
#define _CONTROLLERS

#include "Arduino.h"
#include "Servo12.h"

class Controllers {
private:
    bool sleep;
    bool setting_;
    bool i2c_mode;
private:
    #ifdef _ADAFRUIT_PWMServoDriver_H
    Adafruit_PWMServoDriver * driver;
    #endif
    #ifdef _Servo12
    Servo12 * driver;
    #endif

    float _servo_offsets[12];
    float _val_list[12];
    float _thetas[4][3];
    short direction[12];

public:
    Controllers();
    #ifdef _ADAFRUIT_PWMServoDriver_H
    void SetController(Adafruit_PWMServoDriver* pca);
    #endif
    #ifdef _Servo12
    void SetController(Servo12 *servo12);
    #endif

    void TurnOffController();
    void TurnOnController();

    int getpulse(float theta);

    void getDegreeAngles(float La[4][3]);
    void getDegreeAngles(short num, float La);

    void angleToServo(float La[4][3]);
    void angleToServo(short num, float La);

    void getServoAngles(float theta[4][3]);
    void getServoAngles(short num, float theta[4][3]);

    void servoRotate(float thetas[4][3]);
    void servoRotate(short num, float thetas[3]);
    void servoRotate(short num, float theta);

    void calliservo(short num, float theta);
};

#endif