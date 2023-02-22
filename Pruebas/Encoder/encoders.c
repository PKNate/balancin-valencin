#include <18F4550.h>  
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#fuses HS,NOWDT,NOPROTECT, CPUDIV1,PLL5,NOLVP, NOMCLR
#use delay(CLOCK=20M)
#use RS232(BAUD = 115200, XMIT = PIN_C6, RCV = PIN_C7, BITS = 8, PARITY = N)
#use I2C(MASTER, SDA=PIN_B0, SCL=PIN_B1, SLOW)

//direcciones de los puertos
#byte porta = 0xf80
#byte portb = 0xf81
#byte portc = 0xf82
#byte portd = 0xf83
#byte porte = 0xf84

// Interrupción para leer encoders en el puerto B pines B4-B7

// Variables a usar.
int8 urI=0 ,urD=0;                // Variables para recibir por el puerto UART.
int8 pwm1 = 0, pwm2 = 0;      // Variables para setear el valor PWM de cada motor.
signed int16 posD=0 ,posI=0;               // Posiciones o cuentas del encoder.
int8 puerto=0 ,AB=0 ,AB_1=0 ,CD=0 ,CD_1=0 ,auxAB=0 , auxCD =0; // Variables auxiliares en la lectura de encoders.
int8 cnt = 0;                  // Variable para determinar qué PWM se debe mover según lo recibido en puerto serial.
int8 sensores=0;                 // Variable para lectura de sensores.

#int_rb
void rb_isr()
{
    puerto = PORTB;
    AB =((puerto)&(0x30)) >> 4;
    auxAB=AB^AB_1;
    if(auxAB!=0)
       if(auxAB!=3)
       {
          if(((AB_1<<1)^AB)&(0x02))
             posD++;
          else
             posD--;
       }
    AB_1=AB;
    
    CD =((puerto)&(0xC0)) >> 6;
    auxCD=CD^CD_1;
    if(auxCD!=0)
       if(auxCD!=3)
       {
          if(((CD_1<<1)^CD)&(0x02))
             posI++;
          else
             posI--;
       }
    CD_1 = CD;
}


int main()
{  
   // Seteamos como entradas o salidas.
   //set_tris_a(0b11111111);
   //set_tris_b(0b11110011);
   //set_tris_c(0b01000000);
   //set_tris_d(0b11110000);
   //set_tris_e(0b11111111);
 
   // Seteamos pines 16 y 17 del 4550 como PWM
   setup_timer_2(T2_DIV_BY_1,255,1);
   setup_ccp1(CCP_PWM);       //configurando pwm
   setup_ccp2(CCP_PWM);       //configurando pwm 
   
   // Se habilitan interrupciones.
   enable_interrupts(int_rb);
   enable_interrupts(int_rda);
   enable_interrupts(global);
   
   delay_ms(500);
   //der (b)
   output_low(pin_D0); //front
   output_low(pin_D1);
   set_pwm1_duty((int16)0);
   //izq (a)
   output_low(pin_E0); //front
   output_low(pin_E1);
   set_pwm2_duty((int16)0);
   
   while(true)
   {
      printf("I: %Ld\tD: %Ld\r\n", posI, posD);
      delay_ms(50);
   }
return 0;
}

