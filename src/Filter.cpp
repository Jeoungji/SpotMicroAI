#include <Filter.h>

MovingAverageFilter::MovingAverageFilter(uint16_t size_)
: size(size_)
{
    head = 0;
    curretdatasize = 0;
    sum = 0;
}

void MovingAverageFilter::PutData(float data_)
{
    datas[head] = data_;

    if (curretdatasize < size) curretdatasize++; 

    sum += (double)data_;

    if (head < size ) head++;
    else head = 0;

    if (curretdatasize == size) {
        sum -= datas[head];
    }
}

void MovingAverageFilter::ClearData()
{
    curretdatasize = 0;
    head = 0;
}

float MovingAverageFilter::GetData()
{
    return (float)(sum/curretdatasize);
}