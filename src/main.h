// All Debug Control
#define DEBUGING        1

#define MPU9250DEBUG    0

#if DEBUGING == 0
#undef  MPU9250DEBUG
#define MPU9250DEBUG 0

#endif


#define VFilterSize 100

typedef struct Bfloat {
    bool ok;
    float data;
} Bfloat;

typedef struct Bint {
    bool ok;
    int data;
} Bint;

typedef struct XYZ {
    float x;
    float y;
    float z;
} XYZ;

typedef struct volatag_avf {
  int head;
  float voltage[VFilterSize];
}volatag_avf;

typedef struct footVector {
  float x;
  float y;
  float z;
} footVector;

typedef struct XYZ_c {
  int8_t x;
  int8_t y;
  int8_t z;
} XYZ_c;

typedef struct Senddata {
    int8_t speed_x;
    int8_t speed_y;
    XYZ_c centerAngle;
    XYZ_c imu;
    unsigned char voltage;
    unsigned char mode_CID;
    unsigned char checker;
}SENDDATA;

#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  