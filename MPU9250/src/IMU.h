#ifndef IMU_H
#define IMU_H

extern "C"
{
    #include "stm32f1xx.h"
}

class IMU
{
public:
	IMU();
	~IMU() = default;


private:
    void init();
};

#endif // IMU_H
