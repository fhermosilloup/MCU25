#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_

#include <stdint.h>
void HCSR04_Init(void);
uint16_t HCSR04_Get_Distance(void);
int HCSR04_Available(void);

#endif
