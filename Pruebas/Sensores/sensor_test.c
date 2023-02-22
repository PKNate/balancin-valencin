#include <18F4550.h>
#fuses HS, NOWDT, NOPROTECT, CPUDIV1, PLL1, NOLVP, NOMCLR
#use delay(CRYSTAL=20M, CLOCK=20M)
#use RS232(BAUD = 115200, XMIT = PIN_C6, RCV = PIN_C7, BITS = 8, PARITY = N)
#use I2C(MASTER, I2C1, FAST, stream = SSD1306_STREAM)  // Configuración I2C modo Master 

#include <mpu6050.h>
#include <math.h>
#include <mpu6050.c>

signed int16 ax_offset= -831;
signed int16 ay_offset= -193;
signed int16 az_offset= 881;
signed int16 gx_offset= 730;
signed int16 gy_offset= -122;
signed int16 gz_offset= 47;
signed int16 ax,ay,az;
signed int16 gx,gy,gz;
float timer;
float dt=1;
float ang_x, ang_y, ang_x_full, ang_y_full, accel_ang_x, accel_ang_y;
float height;
float az_m_s2;

float ang_x_prev=0, ang_y_prev=0;
char txt[30];

void readTilt();
void tiltFullRange();

void main()
{ 
   delay_ms(500);
   printf("Hola\r\n");
   setup_timer_0(T0_INTERNAL|T0_DIV_256);
   set_timer0((int16)0);
   InitMpu6050();
   
   while(true)
   {
      readTilt(); 
      tiltFullRange();
      printf("x=%5.2f\r\n",ang_x_full);
      printf("y=%5.2f\r\n",ang_y_full);
      printf("t=%.4f\r\n",dt);
      printf("h=%5.2f\r\n",height);
      delay_ms(500);
   }
}

void readTilt()
{
   timer=get_timer0(); 
   dt=timer/46875;
   set_timer0((int16)0);
   
   ax=GetdataMpu6050(MPU6050_RA_ACCEL_XOUT_H)+ax_offset;
   ay=GetdataMpu6050(MPU6050_RA_ACCEL_YOUT_H)+ay_offset;
   az=GetdataMpu6050(MPU6050_RA_ACCEL_ZOUT_H)+az_offset;
   gx=GetdataMpu6050(MPU6050_RA_GYRO_XOUT_H)+gx_offset;
   gy=GetdataMpu6050(MPU6050_RA_GYRO_YOUT_H)+gy_offset;
   gz=GetdataMpu6050(MPU6050_RA_GYRO_ZOUT_H)+gz_offset;
   
   accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
   accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
   
   ang_x = 0.8*(ang_x_prev+(gx/131)*dt) + 0.2*accel_ang_x;
   ang_y = 0.8*(ang_y_prev+(gy/131)*dt) + 0.2*accel_ang_y;
   
   az_m_s2 = az * (9.81/16384.0);
   height = 10*(9.81-az_m_s2);
   
   ang_x_prev=ang_x;
   ang_y_prev=ang_y;
}

void tiltFullRange()
{
   ang_x_full=(-1)*ang_x;
   ang_y_full=(-1)*ang_y;
   
   if(az<0)
   {
      ang_x_full=ang_x+180;
      ang_y_full=ang_y+180;
   }
   
   else
   {
      if(ang_x_full<0)
      ang_x_full+=360;
      
      if(ang_y_full<0)
      ang_y_full+=360;
   }
}






