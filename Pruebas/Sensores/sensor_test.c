#include <18F4550.h>  
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#fuses HS,NOWDT,NOPROTECT,CPUDIV1,PLL1,NOLVP,NOMCLR
#use delay(CLOCK=20M, CRYSTAL=20M)
#use RS232(BAUD = 115200, XMIT = PIN_C6, RCV = PIN_C7, BITS = 8, PARITY = N)
#use I2C(MASTER, SDA=PIN_B0, SCL=PIN_B1, SLOW)
#include <VL53L0X.h>

#define millis()  (msTimer)
int32 msTimer=0;
#INT_TIMER1
void timer1_isr() {
   set_timer1(64910); // keep period at 1 ms (at 8 MHz)
   msTimer++;
}

#include <VL53L0X.c>

//#define LONG_RANGE
//#define HIGH_SPEED
#define HIGH_ACCURACY

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  setMeasurementTimingBudget(20000);
#endif

void main()
{ 
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
   set_timer1(64910); // keep period at 1 ms (at 8 MHz)
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);
   init();
   setTimeout(200);

   while(true)
   {
      printf("%lu\r\n", readRangeSingleMillimeters());
      if (timeoutOccurred()) { printf(" TIMEOUT\r\n"); }
   }
}






