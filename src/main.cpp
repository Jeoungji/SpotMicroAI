#include <Arduino.h>
#include <main.h>

#include <SPI.h>
#include <MPU9250.h>

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3
#define LED      13

#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  \



MPU9250 mpu(&SPI, SPI_CLOCK, SS_PIN);
uint32_t LoopTimer;

void setup() {
	Serial.begin(115200);

	pinMode(INT_PIN, INPUT);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	if(mpu.auto_init() > 0)
		while(1);
	mpu.init_Kalman();
	delay(1000);

	WAITFORINPUT();
}

uint32_t times = 0;

void loop() {
	mpu.CalKalmanAngle(millis());
   	mpu.read_temp();

	// Serial.print(mpu.accel_data[0]);  Serial.print('\t');
	// Serial.print(mpu.accel_data[1]);  Serial.print('\t');
	// Serial.print(mpu.accel_data[2]);  Serial.print('\t');
	Serial.print(mpu.KalmanAngle.Roll);  Serial.print('\t');
	Serial.print(mpu.KalmanAngle.Pitch);  Serial.print('\t');
	Serial.println("");

	LoopTimer = micros();
  	while(micros() - LoopTimer < 1000000);
}

