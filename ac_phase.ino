#include <SPI.h>
#include <PID_v1.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "MAX.h"


MAX31855_MEM_MAP_t * max_reg;
uint8_t * value[4];

#define MAX_PHASE  15250
#define MIN_PHASE  1

double set_point, input, output;

//double kp = 12500, ki = 150, kd = 5000;
//double kp = 90.213, ki = 53.067, kd = 13.267;/*Calculated from step response*/
double kp = 655.34, ki = 33.067, kd = 100;/*2nd Step response test*/


PID mPID(&input, &output, &set_point,kp,ki,kd,DIRECT);

typedef struct PROFILE{
  double ramp_temp;
  double soak_temp;
  double peak_temp;
  double soak_slope;
  uint8_t reflow_status;
}profile_t;

profile_t reflow_profile;
profile_t reflow_unleaded;
profile_t reflow_leaded;
profile_t piva_heat;

volatile uint8_t state = 0;
uint32_t time_count=0;

uint8_t reflow_status = 0x00;

#define RAMP_UP 0x01
#define SOAK    0x02
#define REFLOW  0x04
#define COOL    0x08
#define IDLE    0x10
void setup()
{
  pinMode(3, INPUT);
  pinMode(13,OUTPUT);
  pinMode(12, INPUT);
  pinMode(11,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  attachInterrupt(1, blink, RISING);
  startTimer(30000);
  Serial.begin(115200);
  
  set_point = 0;
  reflow_unleaded.reflow_status = IDLE;
  reflow_unleaded.ramp_temp = 150.0;
  reflow_unleaded.soak_temp = 180.0;
  reflow_unleaded.soak_slope = .04;
  reflow_unleaded.peak_temp = 230;

  reflow_leaded.reflow_status = IDLE;
  reflow_leaded.ramp_temp = 120.0;
  reflow_leaded.soak_temp = 150.0;
  reflow_leaded.soak_slope = .04;
  reflow_leaded.peak_temp = 185;

  piva_heat.reflow_status = IDLE;
  piva_heat.ramp_temp = 75.0;
  piva_heat.soak_temp = 90.0;
  piva_heat.soak_slope = .00001;
  piva_heat.peak_temp = 95.0;

  reflow_profile = reflow_leaded;
  
  mPID.SetMode(AUTOMATIC);
  mPID.SetControllerDirection(REVERSE);
  mPID.SetOutputLimits(MIN_PHASE,MAX_PHASE);
  mPID.SetSampleTime(133);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE0);
}


volatile uint16_t c = MAX_PHASE;
volatile uint8_t sample = 0, update = 0;
int j = MAX_PHASE - 100;
void loop()
{
  if(Serial.available()>0)
  {
    char input = Serial.read();
    Serial.println(input);
    switch(input)
    {
      case '1':
        reflow_profile.reflow_status = RAMP_UP;
        break;
      case '2':
        reflow_profile.reflow_status = COOL;
        break;
#ifndef TUNE        
      case '3':
        reflow_profile = reflow_leaded;
        break;
      case '4':
        reflow_profile = reflow_unleaded;
        break;
      case '5':
        reflow_profile = piva_heat;
        break;
#else        
      case '3':
        reflow_profile.ramp_temp += 20;
        break;
      case 'p':
        kp*=8;
        break;
      case 'o':
        kp/=8;
        break;
      case 'd':
        kd*=8;
        break;
      case 's':
        kd/=8;
        break;
      case 'i':
        ki*=8;
        break;
      case 'u':
        ki/=8;
        break;
#endif
    }
    Serial.println(c);
  }


  mPID.Compute();
  if(update)
  {
   SELECT();
   uint8_t msb = SPI.transfer(0xff);
   uint8_t lsb = SPI.transfer(0xff);
   DESELECT();
   
   c = (uint16_t) output;
   input = (double)((uint16_t) (msb<<6|lsb>>2)/4.0);
   input-=7.5;
#ifdef TUNE
    if(reflow_profile.reflow_status == RAMP_UP)
    {
      set_point =reflow_profile.ramp_temp;
    }else{
      set_point = 0;
    }
#else
   
   if(input<reflow_profile.ramp_temp && (reflow_profile.reflow_status & RAMP_UP))
   {
     set_point = reflow_profile.ramp_temp;
   }
   if(input>reflow_profile.ramp_temp && (reflow_profile.reflow_status & RAMP_UP))
   {
     set_point+=reflow_profile.soak_slope;
     reflow_profile.reflow_status |= SOAK;
   }
   if(input>reflow_profile.soak_temp && (reflow_profile.reflow_status & SOAK))
   {
     reflow_profile.reflow_status = REFLOW;
     set_point = reflow_profile.peak_temp;
   }
   if((input > reflow_profile.peak_temp -1)&& (reflow_profile.reflow_status & REFLOW))
   {
     reflow_profile.reflow_status = COOL;
     set_point = 0;
   }
   if(input<75 && (reflow_profile.reflow_status & COOL))
   {
     reflow_profile.reflow_status = IDLE;
   }
#endif   
   Serial.print(input,DEC);
   Serial.print("\t");
   Serial.print(reflow_profile.reflow_status,HEX);
   Serial.print("\t");
   Serial.print(c);
   Serial.print("\t");
   Serial.print(kp);
   Serial.print(" ");
   Serial.print(ki);
   Serial.print(" ");
   Serial.println(kd);
  
   update = 0;
  }
}


void blink()
{
  for(int i=0;i<650;i++)
  {
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
  }
  TCNT1 = 0x00;
  OCR1B = c;
  state = 1;
  sample++;
  if(!(sample&15))
  {
    update = 1;
  }
}

void startTimer(uint16_t counts)
{
  TCCR1A = _BV(COM1B1);
  TCCR1B = _BV(CS11)| _BV(WGM12);
  uint32_t clock = 250000;
  //uint16_t counts = clock/(10);
  //249 = 1KHz
  //499 = 500Hz
  OCR1B = counts;//Sampling at 3200Hz for 200Hz of 12 Bit data
  TIMSK1 = _BV(OCIE1B);
}

ISR(TIMER1_COMPB_vect)
{
  if(state)
  {
    PORTB |= _BV(PINB1);
    for(int i=0;i<200;i++)
    {
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    PORTB &= ~_BV(PINB1);
    state = 0;
  }
}



int16_t convertInputTemperature(uint16_t temp)
{
}

int16_t convertCoupleTemp()
{
	int16_t ret;
	if((max_reg->temp_couple & 0x3FFF)>8192)
	{
		ret = (int16_t) -(8192-(max_reg->temp_couple&0x1FFF));
	}else{
		ret = (int16_t) (max_reg->temp_couple) & 0x3fff;
	}
	return ret;
}

int16_t convertCoupleTemp(uint16_t input)
{
	int16_t ret;
	if((input & 0x3FFF)>8192)
	{
		ret = (int16_t) -(8192-(input&0x1FFF));
	}else{
		ret = (int16_t) input&0x3fff;
	}
	return ret;
}


void readMAX31855()
{
	uint8_t dat[4];
	
	SELECT();
	for(int i=0;i<4;i++)
	{
		dat[i] = SPI.transfer(0xff);
	}
	DESELECT();
	max_reg = (MAX31855_MEM_MAP_t *) dat;
}
