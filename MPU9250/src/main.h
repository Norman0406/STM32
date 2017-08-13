#ifndef MAIN_H
#define MAIN_H

void errorMessage(char*, int, char*);
#define error() ErrorHandler(__FILE__, __LINE__)

#endif // MAIN_H
