#ifndef SYSTEM_H
#define SYSTEM_H

#include "IMU.h"

extern "C"
{
    #include <cmsis_os.h>
}

class System {
public:
    System();

    void run();
    void errorHandler(char* file, int line);

private:
    void initSystemClock();
    void initGPIOs();
    void initI2C();

    static void mainThreadStatic(const void* argument);
    void mainThread();

    osThreadId m_defaultTaskHandle;
    IMU m_imu;
};

#endif // SYSTEM_H
