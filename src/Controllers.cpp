#include "Controllers.h"


#define pi 3.141592653589793238462643383

Controllers::Controllers()
{
    setting_ = false;
    // Servo Offset
    _servo_offsets[0] = 180 - 20.3  ;
    _servo_offsets[1] = 90  - 0.3   ;
    _servo_offsets[2] = 90  + 10.8  ;
    _servo_offsets[3] = 1   + 5     ;
    _servo_offsets[4] = 90  - 4     ;
    _servo_offsets[5] = 90  - 0.6   ;
    _servo_offsets[6] = 180 - 11    ;
    _servo_offsets[7] = 90  - 1     ;
    _servo_offsets[8] = 90  + 8     ;
    _servo_offsets[9] = 1   + 16.7  ;
    _servo_offsets[10] = 90 - 3     ;
    _servo_offsets[11] = 90 - 2     ;

    direction[0] = -1;
    direction[1] = -1;
    direction[2] = -1;
    direction[3] = 1;
    direction[4] = 1;
    direction[5] = 1;
    direction[6] = -1;
    direction[7] = -1;
    direction[8] = 1;
    direction[9] = 1;
    direction[10] = 1;
    direction[11] = -1;

    for (int i = 0; i < 12; i++)
        _val_list[i] = 0;

    driver = NULL;
    sleep = false;
}
#ifdef _ADAFRUIT_PWMServoDriver_H
void Controllers::SetController(Adafruit_PWMServoDriver * pca) {
    driver = pca;
    TurnOffController();
    setting_ = true;
}
#endif

#ifdef _Servo12
void Controllers::SetController(Servo12 *servo12) {
    driver = servo12;
    TurnOffController();
    setting_ = true;
}
#endif

void Controllers::TurnOffController() {
    if (!setting_) { Serial.println("Before Setting");  return;  }
    if (!sleep) {
        driver->sleep();
        sleep = true;
    }
}
void Controllers::TurnOnController() {
    if (!setting_) { Serial.println("Before Setting");   return; }
    if (sleep) {
        driver->wakeup();
        sleep = false;
    }
}

int Controllers::getpulse(float theta) {
    #ifdef _ADAFRUIT_PWMServoDriver_H
    return (int)(theta * 3.05555556 + 100);
    #endif

    #ifdef _Servo12
    return (int)(theta * 11.1111111 + 500);
    #endif
}

void Controllers::getDegreeAngles(float La[4][3]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            _thetas[i][j] = La[i][j] * 180 / pi;
}
void Controllers::getDegreeAngles(short num, float La) {
    if (num < 0 || num > 12) return;
    _thetas[num / 3][num % 3] = La * 180 / pi;
}

void Controllers::angleToServo(float La[4][3]) {
    getDegreeAngles(La);
   
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            int _max, _min;
            switch (j) {
            case 0:
                _max = 40;
                _min = -50;
                break;
            case 1:
                _max = 90;
                _min = -90;
                break;
            case 2:
                _max = 130;
                _min = -10;
                break;
            }

            if (_thetas[i][j] > _max) {
                _thetas[i][j] = _max;
            }
            if (_thetas[i][j] < _min) {
                _thetas[i][j] = _min;
            }
        }
    }
    //FL Lower
    _val_list[0] = _servo_offsets[0] + direction[0] * _thetas[0][2];
    //FL Upper
    _val_list[1] = _servo_offsets[1] + direction[1] * _thetas[0][1];
    //FL Shoulder
    _val_list[2] = _servo_offsets[2] + direction[2] * _thetas[0][0];

    //FR Lower
    _val_list[3] = _servo_offsets[3] + direction[3] * _thetas[1][2];
    //FR Upper
    _val_list[4] = _servo_offsets[4] + direction[4] * _thetas[1][1];
    //FR Shoulder
    _val_list[5] = _servo_offsets[5] + direction[5] * _thetas[1][0];

    //BL Lower
    _val_list[6] = _servo_offsets[6] + direction[6] * _thetas[2][2];
    //BL Upper
    _val_list[7] = _servo_offsets[7] + direction[7] * _thetas[2][1];
    //BL Shoulder, Formula flipped from the front
    _val_list[8] = _servo_offsets[8] + direction[8] * _thetas[2][0];

    //BR Lower. 
    _val_list[9] = _servo_offsets[9] + direction[9] * _thetas[3][2];
    //BR Upper
    _val_list[10] = _servo_offsets[10] + direction[10] * _thetas[3][1];
    //BR Shoulder, Formula flipped from the front
    _val_list[11] = _servo_offsets[11] + direction[11] * _thetas[3][0];
}
void Controllers::angleToServo(short num, float La) {
    if (num < 0 || num > 12) return;
    getDegreeAngles(num, La);

    int _max, _min;
    switch (num % 3) {
    case 0:
        _max = 40;
        _min = -50;
        break;
    case 1:
        _max = 90;
        _min = -90;
        break;
    case 2:
        _max = 130;
        _min = -10;
        break;
    }

    if (_thetas[num / 3][num % 3] > _max) {
        _thetas[num / 3][num % 3] = _max;
    }
    if (_thetas[num / 3][num % 3] < _min) {
        _thetas[num / 3][num % 3] = _min;
    }

    _val_list[num] = _servo_offsets[num] + direction[num] * _thetas[num / 3][num % 3];
}

void Controllers::getServoAngles(float theta[4][3]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            theta[i][j] = _val_list[i*3 +j];
}
void Controllers::getServoAngles(short num, float theta[4][3]) {
    for (int i = 0; i < 3; i++) {
        theta[num][i] = _val_list[3 * num + i];
    }
}

void Controllers::servoRotate(float thetas[4][3]) {
    if (!setting_) { Serial.println("Before Setting");  return; }
    angleToServo(thetas);

    for (int i = 0; i < 12; i++) {
        if (_val_list[i] > 180) {
            _val_list[i] = 180;
        }
        if (_val_list[i] <= 0) {
            _val_list[i] = 0;
        }
        driver->setPWM(i, 0, getpulse(_val_list[i]));
    }
}
void Controllers::servoRotate(short num, float thetas[3]) {
    if (!setting_) { Serial.println("Before Setting");  return; }
    if (num < 0 || num + 2 > 12) return;

    for (int i = 0; i < 3; i++) {
        angleToServo(i+num, thetas[i]);
        if (_val_list[i+num] > 180) {
            _val_list[i+num] = 180;
        }
        if (_val_list[i+num] <= 0) {
            _val_list[i+num] = 0;
        }
        driver->setPWM(i+num, 0, getpulse(_val_list[i]));
    }
}
void Controllers::servoRotate(short num, float theta) {
    if (num < 0 || num > 11) return;
    int rotate = 1;
    switch (num) {
    case 0:
    case 1:
    case 5:
    case 6:
    case 7:
    case 8:
        rotate = -1;
    }

    int _max, _min;
    switch (num) {
    case 0:
    case 3:
    case 6:
    case 9:
        _max = 127;
        _min = -10;
        break;
    case 1:
    case 4:
    case 7:
    case 10:
        _max = 70;
        _min = -70;
        break;
    case 2:
    case 5:
    case 8:
    case 11:
        _max = 50;
        _min = -30;
        break;
    }
    if (theta > _max) theta = _max;
    if (theta < _min) theta = _min;
    float value = _servo_offsets[num] + rotate * theta;

    if (value > 180) value = 180;
    if (value < 0) value = 0;

    driver->setPWM(num, 0, getpulse(value));
}

void Controllers::calliservo(short num, float theta) {
    if (num < 0 || num > 11) return;
    int rotate = 1;
    switch (num) {
    case 0:
    case 1:
    case 5:
    case 6:
    case 7:
    case 8:
        rotate = -1;
    }

    float value = _servo_offsets[num] + rotate * theta;
    Serial.println(value);
    if (value > 180) {
        value = 180;
        Serial.println("Over 180");
    }
    if (value < 0) {
        value = 0;
        Serial.println("Under 0");
    }

    driver->setPWM(num, 0, getpulse(value));
}
