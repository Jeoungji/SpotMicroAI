#pragma once
#include <Arduino.h>

class MovingAverageFilter{
private:
    const uint16_t size;
    float datas[sizeof(size)];
    uint16_t head;
    uint16_t curretdatasize;
    double sum;
public:
    MovingAverageFilter(uint16_t size_);
    void PutData(float data);
    void ClearData();
    float GetData();
};

