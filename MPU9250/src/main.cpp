#include "main.h"

#include "System.h"

System system;

int main(void)
{
    system.run();
}

void ErrorHandler(char* file, int line)
{
    system.errorHandler(file, line);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
}
#endif
