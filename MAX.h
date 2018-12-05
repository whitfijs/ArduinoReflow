#ifndef MAX_H_
#define MAX_H_

#include <SPI.h>
#define SS PINB0

#define SELECT() PORTB &= ~_BV(SS)
#define DESELECT() PORTB |= _BV(SS)

typedef struct MAX31855_MEM_MAP
{
	uint16_t temp_couple:	14;
	uint8_t reserved:		1;
	uint8_t fault:			1;
	int16_t temp_int:		12;
	uint8_t reserved0:		1;
	uint8_t scv_fault:		1;
	uint8_t scg_fault:		1;
	uint8_t oc_fault:		1;	
}MAX31855_MEM_MAP_t;


int16_t convertCoupleTemp();
int16_t convertInputTemperature();
void readMAX31855();

//extern MAX31855_MEM_MAP_t * max_reg;



#endif /* MAX_H_ */
