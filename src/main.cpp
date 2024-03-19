#include <main.h>


#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   37 
#define INT_PIN  3
#define LED      13

#define CHECKER 65
#define MODE_MASK 0x0F
#define CID_MASK 0xF0
#define COM_MASK 0xF0

#define A_H 170
#define A_Z  90
#define A_X  110


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
float centerpoint[3] = {33.502508864, 0, -3.648709096};
float centerangle[3] = {0, 0, 0};

float Resetfootpoint[4][4] = {{A_X, -A_H, A_Z,1}, {A_X, -A_H, -A_Z,1}, {-A_X, -A_H, A_Z,1}, {-A_X, -A_H, -A_Z,1}};
float CResetcenter[3] = {33.502508864, 0, -3.648709096};
float CResetangle[3] = {0, 0, 0};
float Resetcenter[3] = {33.502508864, 0, -3.648709096};
float Resetangle[3] = {0, 0, 0};

// float robotangle[3] = {0, 0, 0};
float dotheta[4][3] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
footVector IMU ={0,0,0};


long long Debug_Clock = 0;
int low_voltage_count = 0;
int real_interval = 0;
long long main_clk = 0;
long long Data_clock = 0;


int servopin[12] = {0,1,2,3,4,5,6,7,8,9,10,24};
Servo ser[12];

Servo12 driver(&ser[0], &ser[1], &ser[2],
              &ser[3], &ser[4], &ser[5],
              &ser[6], &ser[7], &ser[8],
              &ser[9], &ser[10], &ser[11]);
Controllers cont;
Kinematic kn;
Matrix mat;
MPU9250 mpu(&SPI, SPI_CLOCK, SS_PIN);
SpotMicro robot;

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
    //centerpoint[2] = 0;
    //centerpoint[0] = 40;
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
    //centerpoint[2] = -5;
    //centerpoint[0] = 40;
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
  pinMode(MPOWER, OUTPUT);
  digitalWrite(MPOWER, HIGH);

  if (!driver.begin(servopin, 500, 2500)) 
    while(1) {
       Serial.println("driver set error");
        delay(5000);
    }
  cont.SetController(&driver);
  Serial.begin(115200);


	pinMode(INT_PIN, INPUT);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

  while(!Lie(footpoint, centerpoint, centerangle)){}

	if(mpu.auto_init() > 0) 
		while(1) {
      Serial.println("mpu set error");
      delay(5000);
    }

	delay(1000);

  //WAITFORINPUT();
  Serial.println("start");
  
  
}

void loop() {

  real_interval = millis()-main_clk;
  main_clk = millis();  
  //ReadCommand();


  if ((millis() - System_Clock) >= INTERVAL_MS) {
    System_Clock = millis();

    mpu.CalKalmanAngle(millis());
    kn.calcIK(dotheta, footpoint, centerangle, centerpoint);
    cont.servoRotate(dotheta);
  }

  #if DEBUGING == 1
  if ((millis() - Debug_Clock) > 100) {
    Debug_Clock = millis();

    Serial.print("  clk : ");
    Serial.print(real_interval);
    Serial.print("  ");

    Serial.print(" Mode : ");
    Serial.print(active_flag);

    Serial.print(" center (");
    mat.SerialPrint(Serial, centerpoint,3);

    Serial.print(" angle (");
    mat.SerialPrint(Serial, centerangle, 3);



    Serial.print( "  IMU : ");
    Serial.print((int8_t)(mpu.KalmanAngle.Pitch));
    Serial.print( ", ");
    Serial.print((int8_t)(mpu.KalmanAngle.Yaw));
    Serial.print("  ");
    Serial.print((int8_t)(mpu.KalmanAngle.Roll));
      // Serial.print(" Pitch ");
      // Serial.print(robotangle[2]);
    //Serial.print(" tehta (");
    //mat.SerialPrint(Serial, dotheta);
    //mat.SerialPrint(Serial, footpoint);

    Serial.println("  ");
  }
  #endif
  
}


