#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>

//            ±àÂë 0 : 11000000
#define CODE_0		0xC0
//            ±àÂë 1 : 11111000
#define CODE_1		0xF8
/*ws2812bµÆÖéÊıÁ¿*/
#define RGBLED_COUNT		8

typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
} tWs2812bCache_TypeDef;

extern tWs2812bCache_TypeDef ileddata[RGBLED_COUNT];

void WS2812b_Set(uint16_t Ws2b812b_NUM, uint8_t r,uint8_t g,uint8_t b);
void WS2812B_Task(void);

#endif
