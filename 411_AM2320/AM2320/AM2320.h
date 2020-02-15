#ifndef AM2320_H
#define AM2320_H
#include "stdint.h"
#define AM2320_ADDR_TX 0xb8
#define AM2320_ADDR_RX 0xb9
#define RX 1
#define TX 0
uint8_t AM2320_ReadData(float *h,float *t);
#endif
