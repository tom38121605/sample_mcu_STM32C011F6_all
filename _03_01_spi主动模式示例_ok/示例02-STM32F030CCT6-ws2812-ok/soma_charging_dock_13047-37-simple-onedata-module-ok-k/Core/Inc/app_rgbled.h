 
#ifndef __APP_RGBLED_H__
#define __APP_RGBLED_H__

#include "main.h"



void led2spi_onedata(uint8_t idatain,  uint8_t *sout);
void led2spi_data(uint8_t *sledin,  uint8_t *sout, uint16_t icount);
void move_color(void);
void rgbled_task(void);

   

#endif /* __APP_RGBLED_H__ */

