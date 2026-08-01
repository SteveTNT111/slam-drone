#ifndef __MYSPI_H_
#define __MYSPI_H_

#include "driver/spi_master.h"

extern spi_device_handle_t Spi2_Handle;

void Spi2_Init(void);
uint8_t Spi2_Transfer_Byte(uint8_t Data);
void Spi2_Write_Data(uint8_t *Data,uint32_t Length);
#endif