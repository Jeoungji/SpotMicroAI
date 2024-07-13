#include <main.h>

#define SPI_CLOCK 5000000  // 8MHz clock works.
#define SS_PIN   37 

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
//float dotheta[4][3] = {{PI/2,PI/2,PI/2}, {PI/2,PI/2,PI/2}, {PI/2,PI/2,PI/2}, {PI/2,PI/2,PI/2}};
footVector IMU ={0,0,0};


long long Debug_Clock = 0;
int real_interval = 0;
long long main_clk = 0;
long long Data_clock = 0;


int servopin[12] = {24,10,9,8,7,6,5,4,3,2,1,0};
PWMServo ser[12];
Servo12 driver(&ser[0], &ser[1], &ser[2],
              &ser[3], &ser[4], &ser[5],
              &ser[6], &ser[7], &ser[8],
              &ser[9], &ser[10], &ser[11]);
Controllers cont;
Kinematic kn;
Matrix mat;
MPU9250 mpu(&SPI, SPI_CLOCK, SS_PIN);
SpotMicro robot(MPOWER);
PointState State;

void ReadCommand();

void FV_Check() {
  for (int i =0;i < 4; i++) {
    if (FV[i].x > 90) FV[i].x = 90;
    else if (FV[i].x < -90) FV[i].x = -90;

    if (FV[i].y > 50) FV[i].y = 50;
    else if (FV[i].y < -50) FV[i].y = -50;
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


void setup() {
  //robot.SetVoltage(VSENSOR);

  // if (!driver.begin(servopin, 500, 2500)) 
  //   while(1) {
  //     Serial.println("driver set error");
  //     delay(5000);
  //   }
  // cont.SetController(&driver);
  Serial.begin(250000);

	while(mpu.auto_init() > 0) {
    Serial.println("mpu set error");
    delay(5000);
  }
  WAITFORINPUT();
  
  // robot.Initialization();
  // delay(1000);
  // Serial.println("start");
  //mpu.MeaurementAccelBias(1000);
}

// void loop() {
//   real_interval = millis()-main_clk;
//   main_clk = millis();  
//   ReadCommand();

//   robot.SensingVoltage(true);

//   if ((millis() - System_Clock) >= INTERVAL_MS) {
//     System_Clock = millis();
//     mpu.CalKalmanAngle(millis());
//     robot.imux = mpu.KalmanAngle.Roll;
//     robot.imuy = mpu.KalmanAngle.Pitch;
//     robot.Activate(State);
//     kn.calcIK(dotheta, State.footpoint, State.centerangle, State.centerpoint);
//     cont.servoRotate(dotheta);
//   }

//   #if DEBUGING == 1
//   if ((millis() - Debug_Clock) > DEBUG_MS) {
//     Debug_Clock = millis();

//     Serial.print("  clk : ");
//     Serial.print(real_interval);
//     Serial.print("  ");

//     // // Serial.print("  v : ");
//     // // Serial.print(voltage);
//     // // Serial.print("  ");

//     Serial.print(" Mode : ");
//     Serial.print(robot.Get_mode());

//     // mat.SerialPrint(Serial, dotheta);


//     Serial.print( "  IMU : ");
//     Serial.print((mpu.KalmanAngle.Pitch));
//     Serial.print( ", ");
//     // Serial.print((int8_t)(mpu.KalmanAngle.Yaw));
//     Serial.print("  ");
//     Serial.print((mpu.KalmanAngle.Roll));
//       // Serial.print(" Pitch ");
//       // Serial.print(robotangle[2]);
//     //Serial.print(" tehta (");
//     //mat.SerialPrint(Serial, dotheta);
//     //mat.SerialPrint(Serial, footpoint);
//     Serial.print("  ");
//     robot.PrintData(XYZ_);
//     robot.PrintData(ABR_);
//     robot.PrintData(POINT_);

//     Serial.println("  ");
//   }
//   #endif
  
// }

void loop() {
  mpu.CalKalmanAngle(millis());
  //Serial.printf("%f, %f, %f\n", mpu.accel_data[0], mpu.accel_data[1], mpu.accel_data[2]);
  delay(3);
  Serial.print( "  IMU : ");
  // Serial.print((mpu.accelangle.Pitch));
  // Serial.print( ", ");
  // Serial.print((mpu.accelangle.Roll));
  // Serial.print( ", ");
  // Serial.print((mpu.accelangle.Yaw));
  // Serial.print( ",   ");
  // Serial.print((mpu.gyroangle.Pitch));
  // Serial.print( ", ");
  // Serial.print((mpu.gyroangle.Roll));
  // Serial.print( ", ");
  // Serial.print((mpu.gyroangle.Yaw));
  // Serial.print( ",   ");
  Serial.print((mpu.KalmanAngle.Pitch));
  Serial.print( ", ");
  Serial.print((mpu.KalmanAngle.Roll));
  Serial.print( ", ");
  Serial.println((mpu.KalmanAngle.Yaw));

  // Serial.print((mpu.accel_lowdata[0]));
  // Serial.print( ", ");
  // Serial.print((mpu.accel_lowdata[1]));
  // Serial.print( ", ");
  // Serial.print((mpu.accel_lowdata[2]));
  // Serial.print( ",  ");
  // Serial.print((mpu.accel_data[0]));
  // Serial.print( ", ");
  // Serial.print((mpu.accel_data[1]));
  // Serial.print( ", ");
  // Serial.println((mpu.accel_data[2]));

  // Serial.print( ", ");
  // Serial.print((mpu.gyro_data[0]));
  // Serial.print( ", ");
  // Serial.print((mpu.gyro_data[1]));
  // Serial.print( ", ");
  // Serial.println((mpu.gyro_data[2]));
  
}


void ReadCommand() {
  char command = 0;
  if (Serial.available()) command = Serial.read();
  else return;

  PointState state;
  robot.ReadSetState(&state);

  switch (command) {
    // XYZ
    case 'k':
    state.centerpoint[2] += 1;
    break;
    case ';':
    state.centerpoint[2] -= 1;
    break;
    case 'o':
    state.centerpoint[0] += 1;
    break;
    case 'l':
    state.centerpoint[0] -= 1;
    break;
    case 'p':
    state.centerpoint[1] += 1;
    break;
    case 'i':
    state.centerpoint[1] -= 1;
    break;

    // ABR
    case 'y':
    state.centerangle[2] += pi / 180 * 1;
    break;
    case 'h':
    state.centerangle[2] -= pi / 180 *1;
    break;
    case 'g':
    state.centerangle[0] += pi / 180 *1;
    break;
    case 'j':
    state.centerangle[0] -= pi / 180 *1;
    break;
    case 'u':
    state.centerangle[1] += pi / 180 *1;
    break;
    case 't':
    state.centerangle[1] -= pi / 180 *1;
    break;

    case '8':
    state= robot.Stay;
    break;

    // Walk
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
    case 'z':
      dotheta[0][0] = dotheta[0][0] + 0.01;
    break;
    case 'x':
      dotheta[0][0] = dotheta[0][0] - 0.01;
    break;
    case 'r':
    for (int i = 0; i < 3; i++) {
      CResetcenter[i] = Resetcenter[i];
      CResetangle[i] = Resetangle[i];
    }
    break;
    case '1':
      robot.Set_mode(1);
    break;
    case '2':
      robot.Set_mode(2);
    break;
    case '3':
      robot.Set_mode(3);
    break;
    case '4':
      robot.Set_mode(4);
    break;
    case '5':
      if (walking_mode == 0) walking_mode = 1;
      else if (walking_mode == 1) walking_mode = 0;
    break;
    case '6':
      robot.Set_mode(6);
    break;

    case '0':
      robot.powerOFF();
      break;
    case '9':
      robot.powerON();
    break;
  }
  robot.InputState(state);
}