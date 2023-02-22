#include <18F4550.h>  
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#fuses HS,NOWDT,NOPROTECT, CPUDIV1,PLL1,NOLVP, NOMCLR
#use delay(CLOCK=20M, CRYSTAL=20M)
#use RS232(BAUD = 115200, XMIT = PIN_C6, RCV = PIN_C7, BITS = 8, PARITY = N)
#use I2C(MASTER, SDA=PIN_B0, SCL=PIN_B1, SLOW)

//direcciones de los puertos
#byte porta = 0xf80
#byte portb = 0xf81
#byte portc = 0xf82
#byte portd = 0xf83
#byte porte = 0xf84

// Direcciones de registros del IMU
#define W_DATA         0xD0
#define R_DATA         0xD1
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define SMPRT_DIV      0x19
#define CONFIG_R       0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define ACCEL_XOUT_L   0x3C
#define ACCEL_YOUT_H   0x3D
#define ACCEL_YOUT_L   0x3E
#define ACCEL_ZOUT_H   0x3F
#define ACCEL_ZOUT_L   0x40
#define TEMP_OUT_H     0x41
#define TEMP_OUT_L     0x42
#define GYRO_XOUT_H    0x43
#define GYRO_XOUT_L    0x44
#define GYRO_YOUT_H    0x45
#define GYRO_YOUT_L    0x46
#define GYRO_ZOUT_H    0x47
#define GYRO_ZOUT_L    0x48

// Registros del Timer
#byte TMR0H = 0xfd7 //TMR0H
#byte TMR0L = 0xfd6 //TMR0L
#byte T0CON = 0xfd5 //T0CON
#byte INTCON= 0xff2 //INTCON

// Variables a usar.
int8 urI=0 ,urD=0;                // Variables para recibir por el puerto UART.
int8 pwm1 = 0, pwm2 = 0;      // Variables para setear el valor PWM de cada motor.
unsigned int8 posD=0 ,posI=0;               // Posiciones o cuentas del encoder.
int8 puerto=0 ,AB=0 ,AB_1=0 ,CD=0 ,CD_1=0 ,auxAB=0 , auxCD =0; // Variables auxiliares en la lectura de encoders.
int8 cnt = 0;                  // Variable para determinar qué PWM se debe mover según lo recibido en puerto serial.
int8 sensores=0;                 // Variable para lectura de sensores.

signed int8 A_data_x[2];      // Lectura Ax del MPU
signed int8 A_data_y[2];      // Lectura Ay del MPU
signed int8 A_data_z[2];      // Lectura Az del MPU
signed int8 G_data_y[2];      // Lectura Gy del MPU

// Funcion para escribir al MPU6050
void MPU6050_write(int add, int data)
{
   i2c_start();
   i2c_write(W_DATA);
   i2c_write(add);
   i2c_write(data);
   i2c_stop();
}

// Función para leer datos del MPU.
int16 MPU6050_read(int add)
{
   int retval;
   i2c_start();
   i2c_write(W_DATA);
   i2c_write(add);
   i2c_start();
   i2c_write(R_DATA);
   retval = i2c_read(0);
   i2c_stop();
   return retval;
}

// Función para iniciar el MPU
void MPU6050_init()
{
   MPU6050_write(PWR_MGMT_1, 0x80);
   delay_ms(100);
   MPU6050_write(PWR_MGMT_1, 0x00);
   delay_ms(100);
   MPU6050_write(CONFIG_R, 0x01);
   delay_ms(10);
   MPU6050_write(GYRO_CONFIG, 0x00);
}

// Función para leer el valor Ax del MPU
void MPU6050_get_Ax()
{
   A_data_x[0] = MPU6050_read(ACCEL_XOUT_H);
   A_data_x[1] = MPU6050_read(ACCEL_XOUT_L);
}

// Función para leer el valor Ay del MPU
void MPU6050_get_Ay()
{
   A_data_y[0] = MPU6050_read(ACCEL_YOUT_H);
   A_data_y[1] = MPU6050_read(ACCEL_YOUT_L);
}

// Función para leer el valor Az del MPU
void MPU6050_get_Az()
{
   A_data_z[0] = MPU6050_read(ACCEL_ZOUT_H);
   A_data_z[1] = MPU6050_read(ACCEL_ZOUT_L);
}

// Función para leer el valor Gy del MPU
void MPU6050_get_Gy()
{
   G_data_y[0] = MPU6050_read(GYRO_YOUT_H);
   G_data_y[1] = MPU6050_read(GYRO_YOUT_L);
}

// Función de interrupción para leer datos cuando llegan al puerto serial.
#int_rda
void rda_isr()
{
      if(cnt == 0)
      {
         urI = getc();//obteniendo el dato
         if(urI>=128)
            {
               output_low(pin_E0);
               output_high(pin_E1);
               urI=urI-128;
               pwm2=urI<<1;
            }
         else
            {
               output_high(pin_E0);
               output_low(pin_E1);
               pwm2=urI<<1;
            }
         set_pwm2_duty(pwm2);   //actualizando el valor del pwm
         cnt = 1;
      }
      
      if(cnt == 1)
      {
      urD = getc();//obteniendo el dato
         if(urD>=128)
            {
               output_low(pin_D0);
               output_high(pin_D1);
               urD=urD-128;
               pwm1=urD<<1;
            }
         else
            {
               output_high(pin_D0);
               output_low(pin_D1);
               pwm1=urD<<1;
            }
         set_pwm1_duty(pwm1);   //actualizando el valor del pwm
         cnt = 0;
      }
}

// Interrupción para leer encoders en el puerto B pines B4-B7
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
             posD--;
          else
             posD++;
       }
    AB_1=AB;
    
    CD =((puerto)&(0xC0)) >> 6;
    auxCD=CD^CD_1;
    if(auxCD!=0)
       if(auxCD!=3)
       {
          if(((CD_1<<1)^CD)&(0x02))
             posI--;
          else
             posI++;
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
   
   // Iniciar el MPU
   MPU6050_init();   // Inicializa MPU6050
   
   T0CON=0xC7; //prescaler asignado a timer0 con relación 1:256
   TMR0L=0;
   
   // Setear variables para cuentas de encoder a 0
   AB=0;
   AB_1=0;
   // Inicio de cuentas encoder derecho a mitad del rango para que no desborde
   posD=127;
   // Setear variables para cuentas de encoder a 0
   CD=0;
   CD_1=0;
   // Inicio de cuentas encoder izquierdo a mitad del rango para que no desborde
   posI=127;
   
   // Seteamos el timer con configuraciones a PWM.
   setup_timer_2(T2_DIV_BY_16,254,1);
   
   // Seteamos pines 16 y 17 del 4550 como PWM
   setup_ccp1(CCP_PWM);       //configurando pwm
   setup_ccp2(CCP_PWM);       //configurando pwm 
   
   // Se habilitan interrupciones.
   enable_interrupts(int_rb);
   enable_interrupts(int_rda);
   enable_interrupts(global);
   
   while(true)
   {
      // Leemos puertos de los sensores de seguidor y detector de obstáculo para pendiente.
      //sensores = (portd & 0x70) >> 4;
      // Leemos los datos del MPU
      MPU6050_get_Ax();
      MPU6050_get_Gy();
      
      // Enviamos bytes al puerto serial
      putc(0xAA);          // ID a la PC
      putc(posD);          // cuentas rueda derecha
      putc(posI);          // cuentas rueda izquierda
      posD = 127;          // Inicio de cuentas encoder derecho a mitad del rango para que no desborde
      posI = 127;          // Inicio de cuentas encoder izquierdo a mitad del rango para que no desborde
      putc(A_data_x[0]);   // Ax high
      putc(A_data_x[1]);   // Ax low
      putc(G_data_y[0]);   // Gy high
      putc(G_data_y[1]);   // Gy low
      putc(sensores);       // Sensores de seguidor y detector de obstáculo
      do
      {
         // Aquí no hacemos nada ya que el puerto serial se lee por interrupción
      }
      while(TMR0L<196); // Espera mientas el timer 0 sea menor 196 para un Ts de 10ms
      TMR0L = 0;  // Reseteo contador del timer0 
   }
return 0;
}

