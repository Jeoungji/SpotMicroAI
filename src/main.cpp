#include <Arduino.h>
#include <main.h>

#include <SPI.h>
#include <MPU9250.h>
#include <Adafruit_PWMServoDriver.h>
#include "Controllers.h"
#include "Kinematics.h"
#include "Matrix.h"

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3
#define LED      13

#define DEBUG_MODE true
#define CHECKER 65
#define MODE_MASK 0x0F
#define CID_MASK 0xF0
#define COM_MASK 0xF0

#define Voltage_sensor 14


#define pi 3.141592653589793238462643383

#define INTERVAL_MS     10

#define A_H 170
#define A_Z  100
#define A_X  110


volatag_avf volageSensor = {0,{11,}};



footVector FV[4] = {{0,0,40},{0,0,40},{0,0,40},{0,0,40}}; // FL, FR, BL, BR

float SideStepLength = 0;
float ForwardStepLength = 0;
float StepHeight = 120;

long long System_Clock = 0;

int walking_mode = 0;
//int t_[4] = {200, 150, 200, 600}; // stay shift stay pull
float t_[4] = {200, 300, 200, 700}; // stay shift stay pull
float RightTimer = 0;
float LeftTimer = (t_[0] + t_[1] + t_[2] + t_[3])/2;

//int t_2[2] = {1000, 3000};// shift pull
float t_2[2] = {400, 1500};// shift pull
float TimerF[4] = {0, (t_2[0] + t_2[1])/4*3, (t_2[0] + t_2[1])/4 ,(t_2[0] + t_2[1])/4*2};
bool status[4] = {false, false, false, false};



int active_flag = 4;
int last_active_flag = 4;
float footpoint[4][4] = {{100, -A_H, 100,1}, {100, -A_H, -100,1}, {-100, -A_H, 100,1}, {-100, -A_H, -100,1}};
float centerpoint[3] = {0, 0, 0};
float centerangle[3] = {0, 0, 0};

float Resetfootpoint[4][4] = {{A_X, -A_H, A_Z,1}, {A_X, -A_H, -A_Z,1}, {-A_X, -A_H, A_Z,1}, {-A_X, -A_H, -A_Z,1}};
float CResetcenter[3] = {36, 0, -10};
float CResetangle[3] = {0, 0, 0};
float Resetcenter[3] = {36, 0, -10};
float Resetangle[3] = {0, 0, 0};

// float robotangle[3] = {0, 0, 0};
float dotheta[4][3] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
footVector IMU ={0,0,0};


long long Debug_Clock = 0;
int low_voltage_count = 0;
int real_interval = 0;
long long main_clk = 0;
long long Data_clock = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Controllers cont(12);
Kinematic kn;
Matrix mat;
MPU9250 mpu(&SPI, SPI_CLOCK, SS_PIN);
SENDDATA senddata = {0,};
char sendbuffer[sizeof(SENDDATA)];

void FV_Check() {
  for (int i =0;i < 4; i++) {
    if (FV[i].x > 90) FV[i].x = 90;
    else if (FV[i].x < -90) FV[i].x = -90;

    if (FV[i].y > 50) FV[i].y = 50;
    else if (FV[i].y < -50) FV[i].y = -50;
  }
}

void ReadCommand() {
  char command = 0;
  if (Serial.available()) command = Serial.read();
  else return;

  switch (command) {
    case 'k':
    CResetcenter[2] += 1;
    break;
    case ';':
    CResetcenter[2] -= 1;
    break;
    case 'o':
    CResetcenter[0] += 1;
    break;
    case 'l':
    CResetcenter[0] -= 1;
    break;
    case 'p':
    CResetcenter[1] += 1;
    break;
    case 'i':
    CResetcenter[1] -= 1;
    break;

    case 'y':
    CResetangle[2] += pi / 180 * 1;
    break;
    case 'h':
    CResetangle[2] -= pi / 180 *1;
    break;
    case 'g':
    CResetangle[0] += pi / 180 *1;
    break;
    case 'j':
    CResetangle[0] -= pi / 180 *1;
    break;
    case 'u':
    CResetangle[1] += pi / 180 *1;
    break;
    case 't':
    CResetangle[1] -= pi / 180 *1;
    break;

    case 'w':
    if (active_flag != 2) break;
    for (int i=0;i < 4; i++) FV[i].x += 5;
    FV_Check();
    break;
    case 's':
    if (active_flag != 2) break;
    for (int i=0;i < 4;i++) FV[i].x -= 5;
    FV_Check();
    break;
    case 'd':
    if (active_flag != 2) break;
    for (int i=0;i < 4;i++) FV[i].y -= 5;
    FV_Check();
    break;
    case 'a':
    if (active_flag != 2) break;
    for (int i=0;i < 4;i++) FV[i].y += 5;
    FV_Check();
    break;
    case 'e':
    if (active_flag != 2) break;
    FV[0].x += 2;
    FV[0].y -= 4;

    FV[1].x -= 2;
    FV[1].y -= 4;

    FV[2].x += 2;
    FV[2].y += 4;

    FV[3].x -= 2;
    FV[3].y += 4;
    FV_Check();
    break;
    case 'q':
    if (active_flag != 2) break;
    FV[0].x -= 2;
    FV[0].y += 4;

    FV[1].x += 2;
    FV[1].y += 4;

    FV[2].x -= 2;
    FV[2].y -= 4;

    FV[3].x += 2;
    FV[3].y -= 4;
    FV_Check();
    break;
    case ' ':
    for (int i=0;i < 4;i++) {
      FV[i].x = 0;
      FV[i].y = 0;
    }
    break;


    case 'r':
    for (int i = 0; i < 3; i++) {
      CResetcenter[i] = Resetcenter[i];
      CResetangle[i] = Resetangle[i];
    }
    break;
    case '1':
      active_flag = 1;
    break;
    case '2':
      if (last_active_flag != 4)
        active_flag = 2;
    break;
    case '3':
      active_flag = 3;
    break;
    case '4':
      active_flag = 4;
    break;
    case '5':
      if (walking_mode == 0) walking_mode = 1;
      else if (walking_mode == 1) walking_mode = 0;
    break;

    case '0':
      active_flag = 0;
    break;
    case '9':
      active_flag = 9;
    break;
  }
}
void ReadCommand_rasp() {
  if (Serial1.available()) {
    String data = Serial1.readString();

    int index = data.indexOf(' '); // mode
    int local_mode = data.substring(0, index).toInt();
    data = data.substring(index+1);

    index = data.indexOf(' '); // com
    int l_com = data.substring(0, index).toFloat();
    data = data.substring(index+1);

    index = data.indexOf(' '); // speed x
    float l_speed_x = data.substring(0, index).toFloat();
    data = data.substring(index+1);
  
    index = data.indexOf(' '); // speed y
    float l_speed_y = data.substring(0, index).toFloat();
    data = data.substring(index+1);

    index = data.indexOf(' '); // speed z
    float l_speed_z = data.substring(0, index).toFloat();
    data = data.substring(index+1);

    index = data.indexOf(' '); // euler x
    float l_euler_x = data.substring(0, index).toFloat();
    data = data.substring(index+1);

    index = data.indexOf(' '); // euler y
    float l_euler_y = data.substring(0, index).toFloat();
    data = data.substring(index+1);

    index = data.indexOf(' '); // euler z
    float l_euler_z = data.substring(0, index).toFloat();
    data = data.substring(index+1);
    
    char buf[100];
    sprintf(buf, "%d  %.2f %.2f %.2f  %.2f %.2f %.2f",local_mode, l_euler_x,l_euler_y, l_euler_z,l_speed_x, l_speed_y,l_speed_z);
    Serial.println(buf);
    switch (l_com) {
      case 2:
        for (int i = 0; i < 3; i++) {
          CResetcenter[i] = Resetcenter[i];
          CResetangle[i] = Resetangle[i];
        }
        break;

    }
    if (local_mode == 1) active_flag = local_mode;
    if (local_mode == 5) {
      if (walking_mode == 0) walking_mode = 1;
      else if (walking_mode == 1) walking_mode = 0;
    }else {
      if (active_flag == 1) active_flag = local_mode;
    }

    CResetangle[0] = l_euler_x *pi / 180;
    CResetangle[1] = l_euler_y * pi / 180;
    CResetangle[2] = l_euler_z * pi / 180;

    FV[0].x = l_speed_x + l_speed_z;
    FV[0].y = l_speed_y - 2*l_speed_z;

    FV[1].x = l_speed_x - l_speed_z;
    FV[1].y = l_speed_y - 2*l_speed_z;

    FV[2].x = l_speed_x + l_speed_z;
    FV[2].y = l_speed_y + 2*l_speed_z;

    FV[3].x = l_speed_x - l_speed_z;
    FV[3].y = l_speed_y + 2*l_speed_z;
    FV_Check();
  }
}
float Voltage() {
  volageSensor.voltage[volageSensor.head] = analogRead(Voltage_sensor) * 3.3 / 1024 * 4.5;
  volageSensor.head++;
  if (volageSensor.head >= VFilterSize) volageSensor.head = 0;

  float sum = 0;
  for (int i = 0; i < VFilterSize; i++)
    sum += volageSensor.voltage[i];

  return sum / VFilterSize;
}
void SendCommand_rasp() {
  senddata.checker = 65;
  senddata.mode_CID = 0;
  senddata.mode_CID |= ((unsigned char)active_flag);
  senddata.voltage = (unsigned char)(Voltage()*10);
  senddata.centerAngle.x = (int8_t)(centerangle[0]*100);
  senddata.centerAngle.y = (int8_t)(centerangle[1]*100);
  senddata.centerAngle.z = (int8_t)(centerangle[2]*100);
  senddata.imu.x = (int8_t)(IMU.x);
  senddata.imu.y = (int8_t)(IMU.y);
  senddata.imu.z = (int8_t)(IMU.z);
  
  float l_speed_x =0;
  float l_speed_y =0;
  for (int i = 0; i < 4; i++) {
    l_speed_x += sqrt(FV[i].x*FV[i].x);
    l_speed_y += sqrt(FV[i].y*FV[i].y);
  }
  senddata.speed_x = (char)(l_speed_x/4);
  senddata.speed_y = (char)(l_speed_y/4);


  memcpy(sendbuffer,&senddata, sizeof(SENDDATA));
  Serial1.write(sendbuffer,sizeof(SENDDATA));
}



void Shift(int i, float POINT[4], float Set[4], footVector fv, float Stime, int t) {

  if (i == 0 || i == 1) {
    POINT[0] = -(fv.x / 2)* cos(Stime * pi / t) + A_X;
  }
  else {
    POINT[0] = -(fv.x / 2)* cos(Stime * pi / t) - A_X;
  }

  if (i == 0 || i == 2) {
    POINT[2] = -(fv.y / 2)* cos(Stime * pi / t) + A_Z;
  }
  else {
    POINT[2] = -(fv.y / 2)* cos(Stime * pi / t) - A_Z;
  }

  POINT[1] = -A_H + ( fv.z* sin(Stime * pi / t));
}

void Pull(int i, float POINT[4], float Set[4], footVector fv, float Stime, int t) {
  if (i == 0 || i == 1) {
    POINT[0] = (fv.x / 2)* cos(Stime * pi / t) + A_X;
  }
  else {
    POINT[0] = (fv.x / 2)* cos(Stime * pi / t) - A_X;
  }

  if (i == 0 || i == 2) {
    POINT[2] = (fv.y / 2)* cos(Stime * pi / t) + A_Z;
  }
  else {
    POINT[2] = (fv.y / 2)* cos(Stime * pi / t) - A_Z;
  }
}

void Walking (float point[4][4], float Set[4][4]) {
  RightTimer += INTERVAL_MS;
  LeftTimer += INTERVAL_MS;

  if (RightTimer <= t_[0]) { // 0 ~ 1000
    //Serial.print("stay1");
  }
  else if (RightTimer - t_[0] <= t_[1]) { // 1000 ~ 2000
    Shift(2, point[2], Set[2], FV[2], RightTimer - t_[0], t_[1]);
    Shift(1, point[1], Set[1], FV[1], RightTimer - t_[0], t_[1]);
    centerpoint[2] = -3;
    centerpoint[0] = 40;
    //Serial.print("shift");
  }
  else if (RightTimer - t_[0] - t_[1] <= t_[2]) { // 2000 ~ 3000
    //Serial.print("stay2");
  }
  else if (RightTimer - t_[0] - t_[1] - t_[2] <= t_[3]) { // 3000 ~ 4000
    Pull(2, point[2], Set[2], FV[2], RightTimer - t_[0] - t_[1] - t_[2], t_[3]);
    Pull(1, point[1], Set[1], FV[1], RightTimer - t_[0] - t_[1] - t_[2], t_[3]);
    //Serial.print("pull ");
  }
  else {
    //Serial.print("Reset");
    RightTimer = 0;
  }

  if (LeftTimer <= t_[0]) { // 0 ~ 1000
    //Serial.print("stay1");
  }
  else if (LeftTimer - t_[0] <= t_[1]) { // 1000 ~ 2000
    Shift(3, point[3], Set[3], FV[3], LeftTimer - t_[0], t_[1]);
    Shift(0, point[0], Set[0], FV[0], LeftTimer - t_[0], t_[1]);
    centerpoint[2] = -20;
    centerpoint[0] = 40;
    //Serial.print("shift");
  }
  else if (LeftTimer - t_[0] - t_[1] <= t_[2]) { // 2000 ~ 3000
    //Serial.print("stay2");
  }
  else if (LeftTimer - t_[0] - t_[1] - t_[2] <= t_[3]) { // 3000 ~ 4000
    Pull(3, point[3], Set[3], FV[3], LeftTimer - t_[0] - t_[1] - t_[2], t_[3]);
    Pull(0, point[0], Set[0], FV[0], LeftTimer - t_[0] - t_[1] - t_[2], t_[3]);
    //Serial.print("pull ");
  }
  else {
    //Serial.print("Reset");
    LeftTimer = 0;
  }
}

void Walking2 (float point[4][4], float Set[4][4]) {
  for (int i = 0; i < 4; i++) {
    TimerF[i] += INTERVAL_MS;
    if (TimerF[i] <= t_2[0]) { // pull
      Shift(i, point[i], Set[i], FV[i], TimerF[i], t_2[0]);
      status[i] = true;
    }
    else if (TimerF[i] - t_2[0] <= t_2[1]) { // shift
      Pull(i, point[i], Set[i], FV[i], TimerF[i] - t_2[0], t_2[1]);
      status[i] = false;
    }
    else {
      TimerF[i] = 0;
    }
  }

  if (status[0]) {
    centerpoint[2] -= 0.9;
    centerpoint[0] -= 2;
  }
  else if (status[1]) {
    centerpoint[2] += 0.9;
    centerpoint[0] -= 2;
  }
  else if (status[2]) {
    centerpoint[2] -= 0.9;
    centerpoint[0] += 2;
  }
  else if (status[3]) {
    centerpoint[2] += 0.9;
    centerpoint[0] += 2;
  }
  if (centerpoint[2] < -10 -21) centerpoint[2] = -10 -21;
  if (centerpoint[2] > -10 +21) centerpoint[2] = -10 +21;
  if (centerpoint[0] < 36 - 30) centerpoint[0] = 36 -30;
  if (centerpoint[0] > 36 + 30) centerpoint[0] = 36 +30;
}

bool SetFootPoint(float point[4][4], float goalpoint[4][4], float split) {
  bool success = true;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j ++) {
      if (abs(point[i][j] - goalpoint[i][j]) > split) {
        if (point[i][j] > goalpoint[i][j]) {
          point[i][j] -= split;
          success = false;
        }
        else if (point[i][j] < goalpoint[i][j]) {
          point[i][j] += split;
          success = false;
        }
      }
      else
        point[i][j] = goalpoint[i][j];
    }
  }
  return success;
}

bool SetBodyAxis(float data[3], float goal[3], float split) {
  bool success = true;
  for (int j = 0; j < 3; j ++) {
    if (abs(data[j] - goal[j]) > split) {
      if (data[j] > goal[j]) {
        data[j] -= split;
        success = false;
      }
      else if (data[j] < goal[j]) {
        data[j] += split;
        success = false;
      }
    }
    else
      data[j] = goal[j];
  }
  return success;
}

bool Lie (float point[4][4], float centerxyz[3], float centerrpy[3] ) {
  float Liepoint[4][4] = {{140, -41, 157,1}, {140, -41, -157,1}, {-40, -41, 157,1}, {-40, -41, -157,1}};

  if (SetBodyAxis(centerxyz, CResetcenter, 0.5)) {
    bool success = SetBodyAxis(centerrpy, CResetangle, 0.005);
    success &= SetFootPoint(point, Liepoint, 0.5);
    return success;
  }
  return false;
}

bool Sit (float point[4][4], float centerxyz[3], float centerrpy[3]) {
  float Sitcenterpoint[3] = {0, 0, 0};
  float Sitcenterangle[3] = {0, 0, 0.84};

  if (SetBodyAxis(centerxyz, Sitcenterpoint,0.5)){
    bool success = SetBodyAxis(centerrpy, Sitcenterangle, 0.005);
    success &= SetFootPoint(point, Resetfootpoint, 0.5);
    return success;
  }
  return false;
}

bool Stay (float point[4][4], float centerxyz[3], float centerrpy[3]) {
    bool success = SetBodyAxis(centerxyz, CResetcenter,0.5);
    success &= SetBodyAxis(centerrpy, CResetangle,0.005);
    success &= SetFootPoint(point, Resetfootpoint,0.5);
    return success;
}

float averageFilter(float *arr, int num, float value) {
  float sum = 0;
  for (int i = 1; i < num; i++) {
    arr[i-1] = arr[i];
    sum += arr[i-1];
  }
  arr[num] = value;
  sum += value;
  return sum/num;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(Voltage_sensor, INPUT);
  pwm.begin();
  pwm.setPWMFreq(60);
  cont.SetController(&pwm);
  Serial.begin(115200);
  Serial1.begin(115200);

	pinMode(INT_PIN, INPUT);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	if(mpu.auto_init() > 0)
		while(1);
	mpu.init_Kalman();
	delay(1000);

  senddata.mode_CID = (unsigned char)active_flag;
  senddata.checker = CHECKER;

  WAITFORINPUT();
  Serial.println("start");
}



void loop() {
  real_interval = millis()-main_clk;
  main_clk = millis();
  if (Voltage() < 9.6) {
    if (low_voltage_count >= 10000) {
      Serial.print("LOW_voltage ");
      Serial.println(Voltage());
      low_voltage_count = 0;
    }
    low_voltage_count++;
    Voltage();
  }

  

  else {
      ReadCommand_rasp();
      ReadCommand();
    if ((millis() - System_Clock) > INTERVAL_MS) {
      System_Clock = millis();

      switch (active_flag) {
      case 1:
        Stay(footpoint, centerpoint, centerangle);
        for (int i=0;i < 4;i++) {
          FV[i].x = 0;
          FV[i].y = 0;
        }
        last_active_flag = 1;
      break;
      case 2:
        if (walking_mode == 0) Walking(footpoint, Resetfootpoint);
        else if (walking_mode == 1 ) Walking2(footpoint, Resetfootpoint);
        last_active_flag = 2;
      break;
      case 3:
        Sit(footpoint, centerpoint, centerangle);
        for (int i=0;i < 4;i++) {
          FV[i].x = 0;
          FV[i].y = 0;
        }
        last_active_flag = 3;
      break;
      case 4:
        Lie(footpoint, centerpoint, centerangle);
        for (int i=0;i < 4;i++) {
          FV[i].x = 0;
          FV[i].y = 0;
        }
        last_active_flag = 4;
      break;
      case 0:
        cont.TurnOffController();
      break;
      case 9:
        cont.TurnOnController();
        active_flag = last_active_flag;
      break;
    }

      kn.calcIK(dotheta, footpoint, centerangle, centerpoint);
      cont.servoRotate(dotheta);
    }

    // if (mpu.update()) {
    //   static uint32_t prev_ms = millis();
    //   if (millis() > prev_ms + 25) {
    //       IMU.y = mpu.getYaw();
    //       IMU.x = mpu.getPitch();
    //       IMU.z = mpu.getRoll();
    //       prev_ms = millis();
    //   }
    // }

    if ((millis() - Data_clock)> 100) {
        Data_clock = millis();
        SendCommand_rasp();
    }
    if ((millis() - Debug_Clock) > 100 && DEBUG_MODE) {
      Debug_Clock = millis();

      Serial.print("  clk : ");
      Serial.print(real_interval);
      Serial.print("  ");

      Serial.print("  voltage : ");
      Serial.print(Voltage());
      Serial.print("  ");

      Serial.print(" Mode : ");
      Serial.print(active_flag);
      //Serial.print(" Mode : ");
      //Serial.print(last_active_flag);

      Serial.print( "  Forward : ");
      Serial.print(FV[1].x);
      Serial.print( "  Side : ");
      Serial.print(FV[1].y);
      Serial.print("  ");
      Serial.print(" center (");
      mat.SerialPrint(Serial, centerpoint,3);

      Serial.print(" angle (");
      mat.SerialPrint(Serial, centerangle, 3);

      Serial.print(" status (");
      for (int i = 0; i < 4; i++) {
        Serial.print(status[i]);
        Serial.print(" ");
      }
      Serial.print(") ");

      Serial.print( "  IMU : ");
      Serial.print((int8_t)(IMU.x));
      Serial.print( ", ");
      Serial.print((int8_t)(IMU.y));
      Serial.print("  ");
      Serial.print((int8_t)(IMU.z));
        // Serial.print(" Pitch ");
        // Serial.print(robotangle[2]);
      //Serial.print(" tehta (");
      //mat.SerialPrint(Serial, dotheta);
      //mat.SerialPrint(Serial, footpoint);

      Serial.println("  ");
    }
  }
}