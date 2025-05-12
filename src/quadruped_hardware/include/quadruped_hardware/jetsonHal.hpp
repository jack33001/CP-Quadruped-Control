#ifndef JETSONHAL_HPP
#define JETSONHAL_HPP

#include <quadruped_hardware/sh2_hal.h>





 
int jetsonOpen(sh2_Hal_t *self);
void jetsonClose(sh2_Hal_t *self);
int jetsonRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
int jetsonWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
uint32_t jetsonGetTimeUs(sh2_Hal_t *self);




#endif  // JETSONHAL_HPP