#include "bsp_init.h"

void BSPInit()
{
    DWT_Init(168);//168Mhz/1Mhz
    BSPLogInit();
}