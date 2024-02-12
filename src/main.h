// All Debug Control
#define DEBUGING        1

#define MPU9250DEBUG    0

#if DEBUGING == 0
#undef  MPU9250DEBUG
#define MPU9250DEBUG 0

#endif




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