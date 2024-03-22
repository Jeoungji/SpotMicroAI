#ifndef _Servo12
#define _Servo12

#include <Arduino.h>
#include <PWMServo.h>

// Servo motor class for Using 12channel servo motor in MCU


class Servo12 {
private:
    PWMServo *servo[12];
    int servo_pin[12];
    int servo_value[12];

    int min;
    int max;

    bool setting;
public:
    Servo12(PWMServo *ser1, PWMServo *ser2, PWMServo *ser3,
            PWMServo *ser4, PWMServo *ser5, PWMServo *ser6,
            PWMServo *ser7, PWMServo *ser8, PWMServo *ser9,
            PWMServo *ser10, PWMServo *ser11, PWMServo *ser12) {
        servo[0] = ser1;
        servo[1] = ser2;
        servo[2] = ser3;
        servo[3] = ser4;
        servo[4] = ser5;
        servo[5] = ser6;
        servo[6] = ser7;
        servo[7] = ser8;
        servo[8] = ser9;
        servo[9] = ser10;
        servo[10] = ser11;
        servo[11] = ser12; 
    }

    bool begin(int pin[12], const int min, const int max) {
        setting = false;

        this->min = min;
        this->max = max;
        for (int i = 0; i < 12; i++) {
            servo_pin[i] = pin[i];
            servo[i]->attach(servo_pin[i], this->min, this->max);
            if (!servo[i]->attached())
                return false;
        }
        setting = true;
        return true;
    }

    void reset() {
        for (int i = 0; i < 12; i++) {
            servo_value[i] = 0;
            //servo[i]->detach();
            servo[i]->attach(servo_pin[i], min, max);
        }
    }

    void sleep() {
        // for (int i = 0; i < 12; i++)
        //     servo[i]->detach();
        setting = false;
    }

    void wakeup() {
        // for (int i = 0; i < 12; i++) {
        //     //servo[i]->attach(servo_pin[i], min, max);
        //     //servo[i]->writeMicroseconds(servo_value[i]);
        // }
        setting = true;
    }

    uint16_t getPWM(uint8_t num, bool off = false) {
        if (!setting) return 0xFF;
        return servo[num]->read();
    }

    uint8_t setPWM(uint8_t num, uint16_t on, uint16_t off) {
        if (!setting) return 0xFF;
        servo[num]->write(off);
        return num;
    }  

    uint8_t setPWM(uint8_t num, float off) {
        if (!setting) return 0xFF;
        servo[num]->write(off);
        return num;
    }   

};
#endif