#pragma once
#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <PWMServo.h>

class Controllers {
private:
    bool sleep;
    bool setting_;
    bool i2c_mode;
private:
    Adafruit_PWMServoDriver * _pca;
    PWMServo* pwmservo[12];

    short numServo;

    float _servo_offsets[12];
    float _val_list[12];
    float _thetas[4][3];
    short direction[12];
public:
    Controllers(short num);

    void SetController(Adafruit_PWMServoDriver* driver);
    void SetController(int pin[12]);

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