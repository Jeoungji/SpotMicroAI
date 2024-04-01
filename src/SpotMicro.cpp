
#include <SpotMicro.h>


SpotMicro::SpotMicro(int power)
{
    _init_ = false;
    movingstatus = 4;
    lastmovingstatus = movingstatus;
    l_time = micros();
    dt = 0;
    if (power != 0) {
        this->power_pin = power;
        pinMode(this->power_pin, OUTPUT);
        digitalWrite(this->power_pin, LOW);
    }
}

bool SpotMicro::Initialization() {
    _init_ = true;
    ForcedInputState(Lie);
    Serial.println("initialization");
    return true;
}

void SpotMicro::powerON() {
    if (!_init_) return;
    if (!power_pin) return;
    digitalWrite(power_pin, HIGH);
}

void SpotMicro::powerOFF() {
    if (!power_pin) return;
    digitalWrite(power_pin, LOW);
}

bool SpotMicro::ForcedInputState(PointState state) {
    if (!_init_) return false;
    for (int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            set.footpoint[i][j] = state.footpoint[i][j];
    for (int i = 0; i < 3; i++) {
        set.centerpoint[i] = state.centerpoint[i];
        set.centerangle[i] = state.centerangle[i];
    }
    return true;
}

bool SpotMicro::VelocityInputState(const float *state, char coordinate, float velocity = 0) {
    if (!_init_) return false;
    float *set_;
    int i_;
    if (coordinate == XYZ_) {
        if (velocity == 0 || velocity > body_max_v) 
            velocity = body_max_v;
        set_ = set.centerpoint;
        i_ = 1;
    }
    else if (coordinate == ABR_) {
        if (velocity == 0 || velocity > body_max_w) 
            velocity = body_max_w;
        set_ = set.centerangle;
        i_ = 1;
    }
    else if (coordinate == POINT_) {
        if (velocity == 0 || velocity > foot_max_v) 
            velocity = foot_max_v;
        set_ = set.footpoint[0];
        i_ = 4;
    }
    else return false;

    float deltaP = velocity * dt; //mm
    bool success = true;

    for (int i = 0; i < i_; i++) {
        float sum = abs(*(set_ +4*i) - *(state+4*i))
                    + abs(*(set_ +4*i +1) - *(state +4*i +1))
                    + abs(*(set_ +4*i +2) - *(state +4*i +2));
        for (int j = 0; j < 3; j++) {
            float mp = abs(*(set_ +4*i +j) - *(state +4*i +j)) / sum * deltaP;
            if (abs(*(set_ +4*i +j) - *(state +4*i +j)) > mp) {
                if (*(set_ +4*i +j) > *(state +4*i +j))
                    *(set_ +4*i +j) = *(set_ +4*i +j) - mp;
                else
                    *(set_ +4*i +j) = *(set_ +4*i +j) + mp;
                success = false;
            }
            else {
                *(set_ +4*i +j) = *(state +4*i +j);
            }
        }
    }
    return success;
}

bool SpotMicro::VelocityInputState(const float **state, char coordinate, float velocity = 0) {
    return VelocityInputState(*state, coordinate, velocity);
}

void SpotMicro::SetVoltage(int pin) {
    voltage_sensor_pin = pin;
    pinMode(voltage_sensor_pin, INPUT);
}

float SpotMicro::SensingVoltage(bool autoOFF) {
    if (!voltage_sensor_pin) return 0;
    float data = analogRead(voltage_sensor_pin) * 3.3 / 1024 * 4.5;
    voltagesensor.PutData(data);
    data = voltagesensor.GetData();
    if (autoOFF && data < 9) {
        if (!power_pin)
            digitalWrite(power_pin, LOW);
        Serial.print(data);
        Serial.println(" LOW Voltage");
        for (int i = 0; i++; i< 100)
            voltagesensor.PutData(data);
        delay(1000);
    }
    return data;
}

bool SpotMicro::Set_mode(uint8_t s_movingstatus) {
    if (!_init_) return false;
    switch (movingstatus) {
    case 1: // stay
        break;
    case 2: // walk
        if (s_movingstatus == 3) return false;
        if (s_movingstatus == 4) return false;
        break;
    case 3: // sit
    case 4: // lie
        if (s_movingstatus == 2) return false;
        break;
    }
    movingstatus = s_movingstatus;
    return true;
}

void SpotMicro::Activate(PointState &state) {
    if (!_init_) return;
    dt = ((float)(millis() - l_time)) / 1000;
    
    switch (movingstatus) {
    case 1:
        if (VelocityInputState(Stay.centerangle, ABR_, 0.5)) {
            VelocityInputState(*Stay.footpoint, POINT_, 800);
            VelocityInputState(Stay.centerpoint, XYZ_, 10);
        }
        break;
    case 3:
        VelocityInputState(*Sit.footpoint, POINT_, 800);
        VelocityInputState(Sit.centerpoint, XYZ_, 10);
        VelocityInputState(Sit.centerangle, ABR_, 1.);
        break;
    case 4:
        if (VelocityInputState(Lie.centerangle, ABR_, 0.1))
            if (VelocityInputState(Lie.centerpoint, XYZ_, 10))
                VelocityInputState(*Lie.footpoint, POINT_, 500);
        break;
    }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            state.footpoint[i][j] = set.footpoint[i][j];
    for (int i = 0; i < 3; i++) {
        state.centerangle[i] = set.centerangle[i];
        state.centerpoint[i] = set.centerpoint[i];
    }
    l_time = millis();
}

void SpotMicro::Shift(const uint8_t leg, float Stime, int t){
    if (leg == 0 || leg == 1)
        set.footpoint[leg][0] = -(Walkvector[leg].x / 2)* cos(Stime * pi / t) + A_X;
    else
        set.footpoint[leg][0] = -(Walkvector[leg].x / 2)* cos(Stime * pi / t) - A_X;

    if (leg == 0 || leg == 2)
        set.footpoint[leg][2] = -(Walkvector[leg].y / 2)* cos(Stime * pi / t) + A_Z;
    else
        set.footpoint[leg][2] = -(Walkvector[leg].y / 2)* cos(Stime * pi / t) - A_Z;

    set.footpoint[leg][1] = -A_H + ( Walkvector[leg].z* sin(Stime * pi / t));
}

void SpotMicro::Pull(uint8_t leg, float Stime, int t) {
  if (leg == 0 || leg == 1)
    set.footpoint[leg][0] = (Walkvector[leg].x / 2)* cos(Stime * pi / t) + A_X;
  else
    set.footpoint[leg][0] = (Walkvector[leg].x / 2)* cos(Stime * pi / t) - A_X;

  if (leg == 0 || leg == 2)
    set.footpoint[leg][2] = (Walkvector[leg].y / 2)* cos(Stime * pi / t) + A_Z;
  else
    set.footpoint[leg][2] = (Walkvector[leg].y / 2)* cos(Stime * pi / t) - A_Z;
}

void SpotMicro::Walking (int16_t walkingtime[4]) {
    for (int i = 0; i < 4; i++) {
        foottimer[i] += INTERVAL_MS;
        if (foottimer[i] <= walkingtime[0]) { // pull
            Shift(i, foottimer[i], walkingtime[0]);
        }
        else if (foottimer[i] - walkingtime[0] - walkingtime[1]
                 <= walkingtime[2]) {
            //Serial.print("stay2");
        }
        else if (foottimer[i] - walkingtime[0] - walkingtime[1] - walkingtime[2]
                 <= walkingtime[3]) { // shift
            Pull(i,  foottimer[i] - walkingtime[0], walkingtime[1]);
        }
        else {
            foottimer[i] = 0;
        }
    }
}

void SpotMicro::Set_Walking_mode(uint8_t mode) {
    uint16_t sum = walkingtime[0] + walkingtime[1] + walkingtime[2] + walkingtime[3];
    switch (mode) {
        case 0:
            foottimer[0] = foottimer[3] = 0;
            foottimer[1] = foottimer[2] = sum/2;
        break;
        case 1:
            foottimer[0] = 0;
            foottimer[1] = sum * 3 / 4;
            foottimer[2] = sum / 4;
            foottimer[3] = sum * 2 / 4;
        break;
    }
}

void SpotMicro::PrintData(char data) { 
    switch (data) {
    case XYZ_:
        mat.SerialPrint(Serial, set.centerpoint, 3);  
        break;
    case ABR_:
        mat.SerialPrint(Serial, set.centerangle, 3);  
        break;
    case POINT_:
        mat.SerialPrint(Serial, set.footpoint);  
        break;
    }
}