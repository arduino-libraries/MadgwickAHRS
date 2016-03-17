#ifndef CommunitationUtils_h
#define CommunitationUtils_h

#include "Arduino.h"

void serialPrintFloatArr(float * arr, int length);
void serialFloatPrint(float f);
void writeArr(void * arr, uint8_t arr_length, uint8_t type_bytes);
void writeVar(void * val, uint8_t type_bytes);


#endif // CommunitationUtils_h