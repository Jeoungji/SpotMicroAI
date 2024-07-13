
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
    set = Stay;
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
            current.footpoint[i][j] = state.footpoint[i][j];
    for (int i = 0; i < 3; i++) {
        current.centerpoint[i] = state.centerpoint[i];
        current.centerangle[i] = state.centerangle[i];
    }
    return true;
}

bool SpotMicro::InputState(PointState state) {
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

bool SpotMicro::ReadSetState(PointState *state) {
    if (!_init_) return false;
    memcpy(state, &set, sizeof(PointState));
    return true;
}

bool SpotMicro::ReadCurrentState(PointState *state) {
    if (!_init_) return false;
    memcpy(state, &current, sizeof(PointState));
    return true;
}

bool SpotMicro::VelocityInputState(const float *state, char coordinate, float velocity = 0) {
    if (!_init_) return false;
    float *set_;
    int i_;
    if (coordinate == XYZ_) {
        if (velocity == 0 || velocity > body_max_v) 
            velocity = body_max_v;
        set_ = current.centerpoint;
        i_ = 1;
    }
    else if (coordinate == ABR_) {
        if (velocity == 0 || velocity > body_max_w) 
            velocity = body_max_w;
        set_ = current.centerangle;
        i_ = 1;
    }
    else if (coordinate == POINT_) {
        if (velocity == 0 || velocity > foot_max_v) 
            velocity = foot_max_v;
        set_ = current.footpoint[0];
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
    uint16_t sum;
    switch (movingstatus) {
    case 1: // stay
        break;
    case 2: // walk
        if (s_movingstatus == 3) return false;
        if (s_movingstatus == 4) return false;
        sum = walkingtime[0] + walkingtime[1] + walkingtime[2] + walkingtime[3];
        foottimer[0] = 0;
        foottimer[1] = sum/2;
        foottimer[2] = sum/2;
        foottimer[3] = 0;
        break;
    case 3: // sit
    case 4: // lie
        if (s_movingstatus == 2) return false;
        break;
    case 6:
    break;
    }
    movingstatus = s_movingstatus;
    return true;
}

void SpotMicro::Balancing(float p) {
    PointState _current;
    ReadCurrentState(&_current);
    float error = - imuy;
    if (error > 5*PI/180 || error < -5*PI/180)
    set.centerangle[2] = _current.centerangle[2] + p*error;

    error = - imux;
    if (error > 5*PI/180 || error < -5*PI/180)
    set.centerangle[0] = _current.centerangle[0] + p*error;
}

void SpotMicro::Activate(PointState &state) {
    if (!_init_) return;
    dt = ((float)(millis() - l_time)) / 1000;
    int16_t d2[4] = {0,0,0,0};
    switch (movingstatus) {
    case 1:
        if (VelocityInputState(set.centerangle, ABR_, 0.5)) {
            VelocityInputState(*set.footpoint, POINT_, 400);
            VelocityInputState(set.centerpoint, XYZ_, 100);
        }
        break;
    case 2:
        Balancing(0.001);
        VelocityInputState(set.centerangle, ABR_, 1);
        VelocityInputState(set.centerpoint, XYZ_, 100);
        Walking(d2);
        break;
    case 3:
        VelocityInputState(*Sit.footpoint, POINT_, 800);
        VelocityInputState(Sit.centerpoint, XYZ_, 10);
        VelocityInputState(Sit.centerangle, ABR_, 1.);
        break;
    case 4:
        if (VelocityInputState(Lie.centerangle, ABR_, 0.5))
            if (VelocityInputState(Lie.centerpoint, XYZ_, 100))
                VelocityInputState(*Lie.footpoint, POINT_, 300);
        break;
    case 6:
        Balancing(0.0005);
        VelocityInputState(set.centerangle, ABR_, 1);
        break;
    }



    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            state.footpoint[i][j] = current.footpoint[i][j];
    for (int i = 0; i < 3; i++) {
        state.centerangle[i] = current.centerangle[i];
        state.centerpoint[i] = current.centerpoint[i];
    }
    l_time = millis();
}

void SpotMicro::Shift(const uint8_t leg, float Stime, int t){
    if (leg == 0 || leg == 1)
        current.footpoint[leg][0] = -(Walkvector[leg].x / 2)* cos(Stime * pi / t) + A_X;
    else
        current.footpoint[leg][0] = -(Walkvector[leg].x / 2)* cos(Stime * pi / t) - A_X;

    if (leg == 0 || leg == 2)
        current.footpoint[leg][2] = -(Walkvector[leg].y / 2)* cos(Stime * pi / t) + A_Z;
    else
        current.footpoint[leg][2] = -(Walkvector[leg].y / 2)* cos(Stime * pi / t) - A_Z;

    current.footpoint[leg][1] = -A_H + ( Walkvector[leg].z* sin(Stime * pi / t));
}

void SpotMicro::Pull(uint8_t leg, float Stime, int t) {
  if (leg == 0 || leg == 1)
    current.footpoint[leg][0] = (Walkvector[leg].x / 2)* cos(Stime * pi / t) + A_X;
  else
    current.footpoint[leg][0] = (Walkvector[leg].x / 2)* cos(Stime * pi / t) - A_X;

  if (leg == 0 || leg == 2)
    current.footpoint[leg][2] = (Walkvector[leg].y / 2)* cos(Stime * pi / t) + A_Z;
  else
    current.footpoint[leg][2] = (Walkvector[leg].y / 2)* cos(Stime * pi / t) - A_Z;
}

void SpotMicro::Walking (int16_t walkTime[4]) {
    //Serial.print(" walk ");
    for (int i = 0; i < 4; i++) {
        foottimer[i] += (uint16_t)(dt*1000);
        if (foottimer[i] <= walkingtime[0]) { // pull
            //Serial.print("A ");
        }
        else if (foottimer[i] -walkingtime[0] <= walkingtime[1]) {
            uint16_t wd = walkingtime[0];
            Shift(i, foottimer[i]-wd, walkingtime[1]);
            //Serial.print("B ");
        }
        else if (foottimer[i] - walkingtime[0] - walkingtime[1]
                 <= walkingtime[2]) {
            //Serial.print("C ");
        }
        else if (foottimer[i] - walkingtime[0] - walkingtime[1] - walkingtime[2]
                 <= walkingtime[3]) { // shift
            uint16_t wd = walkingtime[0] + walkingtime[1] + walkingtime[2];
            Pull(i,  foottimer[i] - wd, walkingtime[3]);
            //Serial.print("D ");
        }
        else {
            foottimer[i] = 0;
        }
    }
    // mat.SerialPrint(Serial, foottimer, 4);
    // Serial.println(" ");
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
        mat.SerialPrint(Serial, current.centerpoint, 3);  
        break;
    case ABR_:
        mat.SerialPrint(Serial, current.centerangle, 3);  
        break;
    case POINT_:
        mat.SerialPrint(Serial, current.footpoint);  
        break;
    }
}