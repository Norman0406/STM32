#ifndef MAIN_H
#define MAIN_H

void _Error_Handler(char*, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif // MAIN_H
