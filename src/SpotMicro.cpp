
#include <SpotMicro.h>


SpotMicro::SpotMicro()
: voltagesensor(100)
{
    _init_ = false;
    movingstatus = 4;
    lastmovingstatus = movingstatus;
    l_time = micros();
}

bool SpotMicro::Initialization() {
    ForcedInputPointState(Lie);
    
    _init_ = true;
}

bool SpotMicro::ForcedInputPointState(PointState state) {
    for (int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            set.footpoint[i][j] = state.footpoint[i][j];
    for (int i = 0; i < 3; i++) {
        set.centerpoint[i] = state.centerpoint[i];
        set.centerangle[i] = state.centerangle[i];
    }
    return true;
}

float SpotMicro::SensingVoltage(int pin) {
    float data = analogRead(pin) * 3.3 / 1024 * 4.5;
    voltagesensor.PutData(data);
    return voltagesensor.GetData();
}

bool SpotMicro::Set_mode(uint8_t s_movingstatus) {
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

void SpotMicro::Activate(bool stabile) {

    if (stabile) {
        
    }
    long long delta_time = (long long)(((float)(micros() - l_time)) / 1000000);
    switch (movingstatus) {
    case 1:

        break;
    }
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