#pragma once

#ifndef _SPOTMICRO
#define _SPOTMICRO

#include <Arduino.h>
#include <main.h>

//defualt foot length [cube]
#define A_H 170
#define A_Z  180/2
#define A_X  220/2

#define MassX -33.502508864
#define MassY 3.648709096

#define MaxStepHigh 40
#define MaxSteplength 90

struct PointState { 
    float footpoint[4][4];
    float centerpoint[3];
    float centerangle[3];
};

struct footVector {
  float x;
  float y;
  float z;
};


class SpotMicro{
private:
    // reset state
    const PointState Stay = {
        {{A_X, -A_H, A_Z, 1}, {A_X, -A_H, -A_Z, 1}, {-A_X, -A_H, A_Z, 1}, {-A_X, -A_H, -A_Z, 1}},
        {33.502508864, 0, -3.648709096},
        {0, 0, 0}
    };
    const PointState Sit = {
        {{A_X, -A_H, A_Z, 1}, {A_X, -A_H, -A_Z, 1}, {-A_X, -A_H, A_Z, 1}, {-A_X, -A_H, -A_Z, 1}},
        {0, 0, 0},
        {0, 0, 0.84}
    };
    const PointState Lie = {
        {{140, -41, 157, 1}, {140, -41, -157, 1}, {-40, -41, 157, 1}, {-40, -41, -157, 1}},
        {0, 0, 0},
        {0, 0, 0}
    };

    // current state
    //float current_jointangle[4][3];
    PointState current;

    // goal state
    PointState set;
    float set_jointanlgle[4][3] = {0,};

    bool _init_;

    //Type of motion
    // 1:Stay 2:Walk 3:Sit 4:Lie
    uint8_t movingstatus;
    uint8_t lastmovingstatus;

    // motion
    long long l_time;
    PointState current_vel = { 0, };
    float max_velocity = 1; // m/s
    float accel = 1; // m/s2

    // Sensor
    MovingAverageFilter voltagesensor;

    // walking
    footVector Walkvector[4] = {{0,0,MaxStepHigh},};
    uint16_t foottimer[4] = {0,};
    uint16_t walkingtime[4] =  {200, 300, 200, 700};
    

public:


    SpotMicro();
    bool Initialization();
    //bool Init_IMU(MPU9250 imu);

    bool ForcedInputPointState(PointState state); // set_state = state

    float SensingVoltage(int pin);
    bool Set_mode(uint8_t s_movingstatus);
    void Activate(bool stabile);
    

    // walking
    void Shift(const uint8_t leg, float Stime, int t);
    void Pull(uint8_t leg, float Stime, int t);
    void Walking (int16_t walkingtime[4]);
    void Set_Walking_mode(uint8_t mode);
};


struct Node {
    PointState data;
    Node* next;

    Node(const PointState& data) 
        : data(data), next(nullptr) {}
};

class Queue {
private:
    Node* head; // 연결 리스트의 첫 번째 노드
    Node* tail; // 연결 리스트의 마지막 노드
    uint16_t count; // 현재 저장된 데이터 수

public:
    Queue() : head(nullptr), tail(nullptr), count(0) {}

    ~Queue() {
        clear();
    }

    bool push(const PointState& data) {
        if (count == 0xFFFF) return false;

        Node* newNode = new Node(data);
        if (tail == nullptr) {
            head = tail = newNode;
        } else {
            tail->next = newNode;
            tail = newNode;
        }
        ++count;
        return true;
    }

    PointState front() {
        if (count == 0) {
            PointState empty;
            for (int i = 0; i < 3; i++)
                empty.centerangle[i] = 0xFFFF;
            return empty;
        }
        return head->data;
    }

    bool pop() {
        if (isEmpty()) {
            return false;
        }
        Node* temp = head;
        head = head->next;
        if (head == nullptr) {
            tail = nullptr;
        }
        delete temp;
        --count;

        return true;
    }

    PointState fornt_pop() {
        PointState data = front();
        pop();
        return data;
    }

    int size() const {
        return count;
    }

    void clear() {
        while (head != nullptr) {
            pop();
        }
    }

    bool isEmpty() const {
        return count == 0;
    }
};

#endif