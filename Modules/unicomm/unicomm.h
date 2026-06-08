#ifndef __UNICOMM_H
#define __UNICOMM_H

#include <stdint.h>

extern int32_t current_x;
extern int32_t current_y;
extern int32_t target_x;
extern int32_t target_y;

void UniComm_UART8_Init(void);
void UniComm_SendArrival(void);

#endif
