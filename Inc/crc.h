#ifndef _CRC16_
#define _CRC16_

//#include "stm32f4xx_hal.h"


/* 
Name : CRC-16 CCITT 
Poly : 0x1021 x^16 + x^12 + x^5 + 1 
Init : 0xFFFF 
Revert: false 
XorOut: 0x0000 
Check : 0x29B1 ("123456789") 
MaxLen: 4095 байт (32767 бит) - обнаружение 
одинарных, двойных, тройных и всех нечетных ошибок 
*/ 
 


uint16_t Crc16_ini( void );
uint16_t Crc16(uint8_t * pcBlock, uint16_t len);
#endif



