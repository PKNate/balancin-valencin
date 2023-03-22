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

///////////////////////////////////////////////////////////////////////////
// Variables de cpp
float Ts = 0.01;
float ppr = 12.0;
float pi_ = 3.141593;
float NR = 34.02;

float KP = 35.0;
float KD = 0.005;

float KPI = 1.4;
float KII = 8.0;

float KPD = 1.4;
float KID = 8.0;

float VM = 10.37;
float VNM = 10.0;

float omega_Avance = -0.1;
float omega_Pendiente = -0.5;

float phi_hor = -0.108;
float alphap = -0.280;

float omegaM = 20.0;

float deg_2_rad = 3.141593/180;
float rad_2_deg = 180/3.141593;
float accel_div_factor = 16384.0;
float gyro_div_factor = 131.0;
float accel_factor = 1/16384.0;
float gyro_factor = 1/131.0;

float c1 = 0.993;

//PROTOTIPOS DE FUNCIONES
unsigned int v_to_pwm(float voltage);


unsigned int flagcom=0,flagfile=0,signo_sal,pwm,posD,posI,ang; //de 8 bits
signed int incD,incI;
// Valores del MPU
signed long int Ax,Gy;
//Variables para filtro complementario y ángulos
float Xa,Yg,phid=phi_hor,phi_1=0.0,phi=0;
float accelx=0,angulox=0,angulox_1=0,c2=1-c1;
// Variables auxiliares
float t=0,iTs=1/Ts,esc=pi_/(2*ppr*NR),escs=127.0/VM;
// Velocidades deseadas
float omegadD=0, omegadI=0;
// Velocidades
float omegaD=0,omegaI=0;
// Errores actuales y anteriores
float eD,eD_1=0,eI,eI_1=0,e,e_1=0;
// omega deseada
float omegadeseada=0;
// Esfuerzos de control
float uD=0,uI=0;
// salida de pwm a motores
unsigned int uRD_cpp,uRI_cpp;

// Ponderación de velocidades
float KVI=1, KVD=1;

//Avance
float omegaA=omega_Avance;

// Sensores
unsigned int sensores;
short int sensor_LI = 0;
short int sensor_LD = 0;
short int sensor_LC = 0;
short int sensor_P = 0;

// Componentes controladores
float proporcional=0,derivativa=0,proporcionalD=0,integralD=0,proporcionalI=0,integralI=0;

/////////////////////////////////////////////////////////////////////////////////

// Variables a usar.
int8 urI=0 ,urD=0;                // Variables para recibir por el puerto UART.
int8 pwm1 = 0, pwm2 = 0;      // Variables para setear el valor PWM de cada motor.
int8 puerto=0 ,AB=0 ,AB_1=0 ,CD=0 ,CD_1=0 ,auxAB=0 , auxCD =0; // Variables auxiliares en la lectura de encoders.
int8 cnt = 0;                  // Variable para determinar qué PWM se debe mover según lo recibido en puerto serial.

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
             posI--;
          else
             posI++;
       }
    CD_1 = CD;
}

int8 readSensors()
{
   int8 helper=0;
   if(!input(PIN_A1))
      helper=helper | 0b00000001;
   if(!input(PIN_A2))
      helper=helper | 0b00000010;   
   if(!input(PIN_A3))
      helper=helper | 0b00000100; 
   
   return helper;
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
   
   printf("Inicio\r\n");
   
   while(true)
   {
      // Leemos puertos de los sensores de seguidor y detector de obstáculo para pendiente.
      //sensores = (portd & 0x70) >> 4;
      // Leemos los datos del MPU
      MPU6050_get_Ax();
      MPU6050_get_Gy();
      sensores=readSensors();
      
      /*
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
      */
      
      incD = 127-posD;
      incI = 127-posI;
      posD = 127;
      posI = 127;
      
      Ax = A_data_x[0];
      Ax = Ax << 8;
      Ax = Ax + A_data_x[1];
      
      Gy = G_data_y[0];
      Gy = Gy << 8;
      Gy = Gy + G_data_y[1];
      
      // Leer sensores de seguidor y de obstáculo.
      // Convertir el valor de los sensores a bits separados.
      
      sensor_LI=(sensores & 0x04)>>2;
      sensor_LC=(sensores & 0x02)>>1;
      sensor_LD=(sensores & 0x01);
      
      /*
      //Seguidor de línea   
      if(sensor_P == false)
      {
         if(phi<phid)
         {
            if(sensor_LI && !sensor_LD)
               if(sensor_LC)   {KVI=0.8; KVD=1.4;} //0.8, 1.2
               else          {KVI=0.7; KVD=1.6;}   //0.7, 1.4
               
            else if(!sensor_LI && sensor_LD)
               if(sensor_LC)   {KVI=1.4; KVD=0.8;}
               else          {KVI=1.6; KVD=0.7;}
                  
            else if(sensor_LI && sensor_LC && sensor_LD)
               {sensor_P = 1; phid=alphap; omegaA=omega_Pendiente;}
                  
            else
               {KVI=1.0; KVD=1.0;}         
         }
         else
            {KVI=1.0; KVD=1.0;}
      }
      else
         {KVI=1.0; KVD=1.0;}
      */
         
      // Calulo velocidad derecha.
           omegaD=incD*esc*iTs;
      // Calculo velocidad izquierda.
           omegaI=incI*esc*iTs;
        // Los valores del MPU los paso a valores flotantes con sus respectivas escalas.
           Xa=-Ax*accel_factor;
           Yg=Gy*gyro_factor;
           // Calculo de filtro complementario
           accelx=Xa*90;//inclinación de -90 a 90 grados de balancin
           angulox=c1*(angulox_1+Yg*Ts)+c2*accelx;//ecuación del filtro, Yg en grados sobre segundo
           angulox_1=angulox;//respaldando valor pasado
           phi=angulox*deg_2_rad;//conversión a rad

         // Calcular el error de angulo
           e_1 = e;
           e=phid-phi;
           
           // Lazo Maestro
           proporcional=KP*e;
           derivativa=KD*(e-e_1)*iTs;
         omegadeseada=proporcional+derivativa;
         omegadeseada=omegadeseada+omegaA;
         //omegadeseada=15*sin(6.28*t);
         //omegadeseada=6.28;
         //Saturación de omega_d
         if(omegadeseada > omegaM)
            omegadeseada = omegaM;
         if(omegadeseada < -omegaM)
            omegadeseada = -omegaM;
         
         omegadI = KVI*omegadeseada;
         omegadD = KVD*omegadeseada;
         
         //omegadI = omegaM*sin(0.628*t);
         //omegadD = omegaM*cos(0.628*t);
         
         //omegadI = 6.28;
         //omegadD = -6.28;
         
         //Lazo esclavo izquierdo
         eI_1 = eI;               
         eI=omegadI-omegaI;
         proporcionalI=KPI*eI;
         if((integralI<VM)&&(integralI>(-VM)))
                 integralI=integralI+KII*Ts*eI;
              else
                {
              if(integralI>=VM)
               integralI=0.95*VM;
              if(integralI<=(-VM))
                integralI=-0.95*VM;
             }
         uI=proporcionalI+integralI;
         //uI=VNM*sin(0.3141*t);
             //uI=0;
             //uI=2.0;
         if(uI>=VM)
               uI=VM;
           if(uI<=(-VM))
                uI=-VM;
             uRI = v_to_pwm(uI);
             
             //Lazo esclavo derecho
         eD_1 = eD;               
         eD=omegadD-omegaD;
         proporcionalD=KPD*eD;
         if((integralD<VM)&&(integralD>(-VM)))
                 integralD=integralD+KID*Ts*eD;
              else
                {
              if(integralD>=VM)
               integralD=0.95*VM;
              if(integralD<=(-VM))
                integralD=-0.95*VM;
             }
         uD=proporcionalD+integralD;
         //uD=VM*sin(0.3141*t);
             //uD=0;
             //uD=2.0;
         if(uD>=VM)
               uD=VM;
           if(uD<=(-VM))
                uD=-VM;
             uRD = v_to_pwm(uD);

          t=t+Ts;//t+=Ts;
      
         if(urI>=128)
            {
               output_low(pin_E1);
               output_high(pin_E0);
               urI=urI-128;
               pwm2=urI<<1;
            }
         else
            {
               output_high(pin_E1);
               output_low(pin_E0);
               pwm2=urI<<1;
            }
         set_pwm2_duty(pwm2);   //actualizando el valor del pwm

         if(urD>=128)
            {
               output_low(pin_D1);
               output_high(pin_D0);
               urD=urD-128;
               pwm1=urD<<1;
            }
         else
            {
               output_high(pin_D1);
               output_low(pin_D0);
               pwm1=urD<<1;
            }
         set_pwm1_duty(pwm1);   //actualizando el valor del pwm
      
       //Imprimiendo en pantalla
       printf("%.3f\r\n",phi);
       //printf("%i\t\n", pos);
       //printf("%.2f\t%.2f\t%.2f\n",t,uI,uD);
       //printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\t%d\n",t,uI,uD,omegaI,omegaD,incI,incD);
       //printf("%.2f\t%d\t%d\n",t,Ax,Gy);
       //printf("%.3f\t%.3f\t%.3f\t%.3f\n",phid, phi, uI, uD);
       //printf("%.2f\t%.2f\t%.2f\t%d\t%d\t%d\t%d\n",t,phid,phi,sensor_LI, sensor_LC, sensor_LD, sensor_P);
               
      
      if (TMR0L > 196)
      {
         output_high (PIN_D2);
         output_high (PIN_D3);
      
      }
      
      else
      {
      do
         {
            // Aquí no hacemos nada ya que el puerto serial se lee por interrupción
         }
         while(TMR0L<196); // Espera mientas el timer 0 sea menor 196 para un Ts de 10ms
      }
      TMR0L = 0;  // Reseteo contador del timer0 
   }
return 0;
}

unsigned int v_to_pwm(float voltage)
{
   float pwmf;
   pwmf=escs*voltage;
   pwm=(unsigned char)fabs(pwmf);
   if(pwmf<0)
      pwm=pwm+128;
   return pwm;
}

