/* Modelo en PC (NO es hardware) de 2 AXI Quad SPI + LSM9DS1, para probar la logica de main.c sin placa. Los tiempos NO valen. */
#pragma once
#include <stdint.h>
uint32_t Xil_In32(uint32_t addr);
void     Xil_Out32(uint32_t addr, uint32_t val);
uint64_t gt_now(void);
double   host_gt_hz(void);
