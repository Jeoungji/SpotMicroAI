#pragma once

#ifndef _FILTER
#define _FILTER

#define AVERAGEFILTERSIZE 200

class MovingAverageFilter{
private:
    const uint16_t size;
    float datas[AVERAGEFILTERSIZE];
    uint16_t head;
    uint16_t curretdatasize;
    double sum;
public:
    MovingAverageFilter() 
    : size(AVERAGEFILTERSIZE-1), head(0), curretdatasize(0), sum(0){    }
    void PutData(float data_)
    {
        datas[head] = data_;

        if (curretdatasize < size)
        curretdatasize += 1; 

        sum = sum + (double)data_;

        if (head < size )
        head += 1;
        else 
        head = 0;

        if (curretdatasize == size)
            sum = sum - (double)datas[head];
    }
    void ClearData() {
        curretdatasize = 0;
        head = 0;
    }   

    float GetData() { return (float)(sum/curretdatasize); }

    bool Full() {
        if (curretdatasize == size) return true;
        return false;
    }
    uint16_t Size() { return curretdatasize; }
};

#endif