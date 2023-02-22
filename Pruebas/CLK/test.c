#include <18F4550.h> 
#fuses HS, NOWDT, NOPROTECT, CPUDIV1, PLL1, NOLVP, NOMCLR
#use delay(CRYSTAL=20M, CLOCK=20M)
#use RS232(BAUD = 115200, XMIT = PIN_C6, RCV = PIN_C7, BITS = 8, PARITY = N)


void main()
{  
   output_high(PIN_D2);
   output_low(PIN_D3);

   while(true)
   {
      output_toggle(PIN_D2);
      output_toggle(PIN_D3);
      printf("Hola");
      delay_ms(1000);
   
   }

}
