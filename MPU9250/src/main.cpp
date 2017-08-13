#include "main.h"

#include "System.h"

System system;

int main(void)
{
    return system.run();
}

void errorMessage(char* file, int line, char* message)
{
    system.errorHandler(file, line, message);
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
