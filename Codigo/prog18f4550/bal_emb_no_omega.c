#include <18F4550.h>  
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#fuses HS,NOWDT,NOPROTECT,CPUDIV1,PLL1,NOLVP,NOMCLR
#use delay(CLOCK=20M, CRYSTAL=20M)
#use RS232(BAUD = 115200, XMIT = PIN_C6, RCV = PIN_C7, BITS = 8, PARITY = N)
#use I2C(MASTER, SDA=PIN_B0, SCL=PIN_B1, SLOW)

// ---------------------- Direcciones de los puertos --------------------------
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

// --------------------------- Variables de CPP -------------------------------
float Ts = 0.01;
float ppr = 12.0;
float pi_ = 3.141593;
float NR = 34.02;

float KP = 35.0;     //35.0;
float KD = 0.53;     //0.005;
float KI = 158.0;    //158.0;

float KPI = 2.0;     //2.0 //1.4;
float KII = 8.0;     //8.0;

float KPD = 2.0;     //1.4;
float KID = 8.0;     //2.0 //8.0;

float KPP = 0.05;
float KDP = 0.0001;
float KIP = 0.01;
float omegaP_d = 2.0;

//-0.1495; //-0.1485; //-0.1435; //-0.1375; //-0.1335;//-0.108;
float phi_max = -0.1495;
float phi_min = -0.5330;

float VM = 10.37;
float omegaM = 20.0;

float deg_2_rad = 3.141593/180;
float accel_factor = 1/16384.0;
float gyro_factor = 1/131.0;

// Cambio para que coincida con el marco teorico
float cf = 0.007;
float c1 = 1 - cf;

// Funcion voltaje a pwm
unsigned int v_to_pwm(float voltage);

unsigned int pwm,posD,posI; //de 8 bits
signed int incD,incI;
// Valores del MPU
signed long int Ax,Gy;
// Variables para filtro complementario y ángulos
float Xa, Yg, phid=phi_max, phi=0;
float accelx=0, angulox=0, angulox_1=0;
// Variables auxiliares
float iTs=1/Ts, esc=pi_/(2*ppr*NR), escs=127.0/VM;
// Velocidades deseadas
float omegadD=0, omegadI=0;
// Velocidades
float omegaD=0,omegaI=0,omegaP=0;
// Errores actuales y anteriores
float eD,eD_1=0,eI,eI_1=0,e,e_1=0,eP,eP_1=0; 
// Omega deseada
float omegadeseada=0;
// Esfuerzos de control
float uD=0,uI=0;
// Ponderacion de velocidades
float KVI=1, KVD=1;

// Sensores
unsigned int sensores;
short int sensor_LI = 0;
short int sensor_LD = 0;
short int sensor_LC = 0;
short int sensor_P = 0;

// Componentes controladores
float proporcional=0, derivativa=0, proporcionalD=0, integralD=0,
proporcionalI=0, integralI=0, proporcionalP=0, derivativaP=0, integralP=0;
float integral = 0;

// ---------------------------- Variables a usar ------------------------------
int8 urI=0 ,urD=0;                                             
int8 pwm1 = 0, pwm2 = 0;                                       
int8 puerto=0 ,AB=0 ,AB_1=0 ,CD=0 ,CD_1=0 ,auxAB=0 , auxCD =0; 

signed int8 A_data_x[2];                                       
signed int8 A_data_y[2];                                       
signed int8 A_data_z[2];                                       
signed int8 G_data_y[2];                                       

// Variables para el escalado.
signed int8 phi_esc=0, phid_esc=0, omegadI_esc=0, omegaI_esc=0, omegadD_esc=0,
omegaD_esc=0, omegaP_esc=0, uI_esc=0, uD_esc=0;
float temp;

// Funcion para escribir al MPU6050
void MPU6050_write(int add, int data)
{
   i2c_start();
   i2c_write(W_DATA);
   i2c_write(add);
   i2c_write(data);
   i2c_stop();
}

// Funcion para leer datos del MPU.
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

// Funcion para iniciar el MPU
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

// Funcion para leer el valor Ax del MPU
void MPU6050_get_Ax()
{
   A_data_x[0] = MPU6050_read(ACCEL_XOUT_H);
   A_data_x[1] = MPU6050_read(ACCEL_XOUT_L);
}

// Funcion para leer el valor Ay del MPU
void MPU6050_get_Ay()
{
   A_data_y[0] = MPU6050_read(ACCEL_YOUT_H);
   A_data_y[1] = MPU6050_read(ACCEL_YOUT_L);
}

// Funcion para leer el valor Az del MPU
void MPU6050_get_Az()
{
   A_data_z[0] = MPU6050_read(ACCEL_ZOUT_H);
   A_data_z[1] = MPU6050_read(ACCEL_ZOUT_L);
}

// Funcion para leer el valor Gy del MPU
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
   // Iniciar el MPU
   MPU6050_init();
   
   // Prescaler asignado a timer0 con relación 1:256
   T0CON = 0xC7; 
   TMR0L = 0;
   
   // Setear variables para cuentas de encoder a 0
   AB=0;
   AB_1=0;
   // Inicio de cuentas encoder derecho a mitad del rango
   posD=127;
   // Setear variables para cuentas de encoder a 0
   CD=0;
   CD_1=0;
   // Inicio de cuentas encoder izquierdo a mitad del rango
   posI=127;
   
   // Seteamos el timer con configuraciones a PWM.    // 1225 Hz
    setup_timer_2(T2_DIV_BY_16,254,1);

   // Seteamos pines 16 y 17 del 4550 como PWM
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   
   // Se habilitan interrupciones.
   enable_interrupts(int_rb);
   enable_interrupts(int_rda);
   enable_interrupts(global);

   while(true)
   {
// ----------------------- Lectura de sensores --------------------------------
      // Leemos sensores
      MPU6050_get_Ax();
      MPU6050_get_Gy();
      sensores=readSensors();
      
      // Ajuste de datos de MPU
      Ax = A_data_x[0];
      Ax = Ax << 8;
      Ax = Ax + A_data_x[1];
      
      Gy = G_data_y[0];
      Gy = Gy << 8;
      Gy = Gy + G_data_y[1];
      
      // Ajuste de datos de seguidores de linea
      sensor_LI=(sensores & 0x04)>>2;
      sensor_LC=(sensores & 0x02)>>1;
      sensor_LD=(sensores & 0x01);
      
      // Resetear cuentas a la mitad
      incD = 127-posD;
      incI = 127-posI;
      posD = 127;
      posI = 127;

// ------------------------- Calculos requeridos ------------------------------
      // Calculo velocidad derecha.
      omegaD=incD*esc*iTs;
      // Calculo velocidad izquierda.
      omegaI=incI*esc*iTs;
      // Los valores del MPU los paso a valores flotantes.
      Xa=-Ax*accel_factor;
      Yg=Gy*gyro_factor;
      // Calculo de filtro complementario
      // Inclinacion de -90 a 90 grados
      accelx=Xa*90;
      // Ecuacion del filtro, Yg en grados sobre segundo
      angulox=c1*(angulox_1+Yg*Ts)+cf*accelx;
      // Respaldando valor pasado
      angulox_1=angulox;
      // Conversion a rad
      phi=angulox*deg_2_rad;
      // Promedio velocidad
      omegaP = ((omegaD / KVD) + (omegaI / KVI))/2.0;
      // Calcular el error de omega
      eP_1 = eP;
      eP = omegaP_d-omegaP;
      eP = eP * (-1.0);
      
// -------------------------- Lazo Inclinacion --------------------------------   
      proporcionalP=KPP*eP;
      derivativaP=KDP*(eP-eP_1)*iTs;
      if((integralP < phi_max) && (integralP > phi_min))
         integralP=integralP+KIP*Ts*eP;
      else
      {
         if(integralP>=phi_max)
            integralP=0.95*phi_max;
         if(integralP<=phi_min)
            integralP=-0.95*phi_min;
      }
      phid=proporcionalP+derivativaP+integralP;
      //Saturacion de phid
      if(phid > phi_max)
         phid = phi_max;
      if(phid < phi_min)
         phid = phi_min;
         
      // Calcular el error de angulo
      e_1 = e;
      e=phid-phi;    
 
// ------------------------- Seguidor de linea --------------------------------  
      if(sensor_LI && !sensor_LD)
         if(sensor_LC)   {KVI=0.7; KVD=1.8;}   //0.8, 1.4
         else            {KVI=0.4; KVD=2.2;}   //0.7, 1.6
         
      else if(!sensor_LI && sensor_LD)
         if(sensor_LC)   {KVI=1.8; KVD=0.7;}
         else            {KVI=2.2; KVD=0.4;}
                   
      else
         {KVI=1.0; KVD=1.0;}  
     
// -------------------------- Lazo Maestro ------------------------------------
      proporcional=KP*e;
      derivativa=KD*(e-e_1)*iTs;
      if((integral<VM)&&(integral>(-VM)))
         integral=integral+KI*Ts*e;
      else
      {
         if(integral>=VM)
            integral=0.95*VM;
         if(integral<=(-VM))
            integral=-0.95*VM;
      }
      omegadeseada=proporcional+derivativa+integral;
      //omegadeseada=omegadeseada+omegaA;
      //omegadeseada=15*sin(6.28*t);
      //omegadeseada=6.28;
      
      //Saturacion de omega deseada
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
      
// -------------------------- Lazo Esclavo I ----------------------------------
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
          
// -------------------------- Lazo Esclavo D ----------------------------------
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

// -------------------------- Salida a PWM ------------------------------------
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
      // Actualizando el valor del pwm
      set_pwm2_duty(pwm2);

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
      // Actualizando el valor del pwm
      set_pwm1_duty(pwm1);
          
// --------------------- Escalado de variables --------------------------------
      temp = 128 * phi;
         if (temp < -128 )
            phi_esc = -128;
         else if (temp > 127)
            phi_esc = 127;
         else
            phi_esc = (signed int8)temp;
      
      temp = 128 * phid;
         if (temp < -128 )
            phid_esc = -128;
         else if (temp > 127)
            phid_esc = 127;
         else
            phid_esc = (signed int8)temp;
      
      temp = (3.1875 * omegadI) - 0.5;
         if (temp < -128 )
            omegadI_esc = -128;
         else if (temp > 127)
            omegadI_esc = 127;
         else
            omegadI_esc = (signed int8)temp;
      
      temp = (3.1875 * omegaI) - 0.5;
         if (temp < -128 )
            omegaI_esc = -128;
         else if (temp > 127)
            omegaI_esc = 127;
         else
            omegaI_esc = (signed int8)temp;
            
      temp = (3.1875 * omegadD) - 0.5;
         if (temp < -128 )
            omegadD_esc = -128;
         else if (temp > 127)
            omegadD_esc = 127;
         else
            omegadD_esc = (signed int8)temp;
      
      temp = (3.1875 * omegaD) - 0.5;
         if (temp < -128 )
            omegaD_esc = -128;
         else if (temp > 127)
            omegaD_esc = 127;
         else
            omegaD_esc = (signed int8)temp;
      
      temp = (3.1875 * omegaP) - 0.5;
         if (temp < -128 )
            omegaP_esc = -128;
         else if (temp > 127)
            omegaP_esc = 127;
         else
            omegaP_esc = (signed int8)temp;     
           
      temp = (8.5 * uI) - 0.5;
         if (temp < -128 )
            uI_esc = -128;
         else if (temp > 127)
            uI_esc = 127;
         else
            uI_esc = (signed int8)temp;
       
      temp = (8.5 * uD) - 0.5;
         if (temp < -128 )
            uD_esc = -128;
         else if (temp > 127)
            uD_esc = 127;
         else
            uD_esc = (signed int8)temp;
      
      sensores = sensores + (sensor_P * 8);
      
// ----------------------------- Salida a PC ----------------------------------
      putc(0xAA);
      putc(phi_esc);          
      putc(omegadI_esc);      
      putc(omegaI_esc);       
      putc(omegadD_esc);      
      putc(omegaD_esc);       
      putc(uI_esc);           
      putc(uD_esc);           
      putc(sensores);
      putc(phid_esc);
      putc(omegaP_esc);

// ------------------------- Tiempo de muestreo -------------------------------     
      //Indicador de sobrepase de Ts
      if (TMR0L > 196)        
      {
         output_high (PIN_D2);
         output_high (PIN_D3);
      }
      
      // Espera mientas el timer 0 sea menor 196 para un Ts de 10ms
      else
      {
         do{}
         while(TMR0L < 196); 
      }
      TMR0L = 0;
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

