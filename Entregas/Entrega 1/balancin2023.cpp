#include <iostream>
#include <string.h>
#include <dos.h>
#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <conio.h>
#include <stdlib.h>
#include <stdbool.h> 

#define Ts      0.01
#define ppr     12.0 	//48 vueltas por revolucion (12*4)
#define pi_     3.141593
#define NR      34.02

#define KP  35.0			//40.0 //45.0
#define KD  0.005			//0.01

#define KPI  1.4			//1.4
#define KII  8.0 			//8.0

#define KPD  1.4			//1.4
#define KID  8.0			//8.0

#define VM     10.37		//Poner capacidad real
#define VNM    10.0		    //Cuanto es lo máximo que quiero mandar

#define omega_Avance -0.1		//Avance (negativo -> hacia delante) //-1.0 //-10.0 //-15.0 //-2.0
#define omega_Pendiente -0.5	//-10

#define phi_hor -0.108		//-0.123 //-0.080 //-0.138 //-0.160 //-0.400 //-0.115
#define alphap -0.280		//Inclinación //-0.200 //-0.330

#define omegaM 20.0			//Velocidad máxima (rad/s)

#define deg_2_rad 			pi_/180 
#define rad_2_deg 			180/pi_
#define accel_div_factor 	16384.0
#define gyro_div_factor 	131.0
#define accel_factor 		1/accel_div_factor
#define gyro_factor 		1/gyro_div_factor

#define cf 0.007

//PROTOTIPOS DE FUNCIONES
unsigned char v_to_pwm(float voltage);



unsigned char flagcom=0,flagfile=0,signo_sal,pwm,posD,posI,ang; //de 8 bits
signed char incD,incI;
// Valores del MPU
signed short int Ax,Gy;
//Variables para filtro complementario y ángulos
float Xa,Yg,phid=phi_hor,phi_1=0.0,phi=0;
float accelx=0,angulox=0,angulox_1=0,c1=1-cf;
// Variables auxiliares
float t=0,iTs=1/Ts,esc=pi_/(2*ppr*NR),escs=127.0/VM;
// Velocidades deseadas
float omegadD=0, omegadI=0;
// Velocidades
float omegaD=0,omegaI=0;
// Errores actuales y anteriores
float eD,eD_1=0,eI,eI_1=0,e,e_1=0;
// omega deseada
float omegad=0;
// Esfuerzos de control
float uD=0,uI=0;
// salida de pwm a motores
unsigned char uRD,uRI;

// Ponderación de velocidades
float KVI=1, KVD=1;

//Avance
float omegaA=omega_Avance;

// Sensores
unsigned int sensores;
bool sensor_LI = false;
bool sensor_LD = false;
bool sensor_LC = false;
bool sensor_P = false;

// Componentes controladores
float proporcional=0,derivativa=0,proporcionalD=0,integralD=0,proporcionalI=0,integralI=0;


using namespace std;
int main()
{
    HANDLE h; /*handler, sera el descriptor del puerto*/
    DCB dcb; /*estructura de configuracion*/
    DWORD dwEventMask; /*mascara de eventos*/
    FILE *fp;
    
    if((fp=fopen("datos.txt","w+"))==NULL)
	{
	  printf("No se puede abrir el archivo.\n");
	  exit(1);
	}    
        
        
    /*abrimos el puerto*/
    h=CreateFile("COM3",GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    
    if(h == INVALID_HANDLE_VALUE) 
	{
         /*ocurrio un error al intentar abrir el puerto*/
    }
             
	/*obtenemos la configuracion actual*/
	if(!GetCommState(h, &dcb)) 
	{
         /*error: no se puede obtener la configuracion*/
        printf("No se puede abrir");
    }
         
    /*Configuramos el puerto*/
	dcb.BaudRate = 115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = TRUE;
         
    /* Establecemos la nueva configuracion */
    if(!SetCommState(h, &dcb)) 
	{
        /* Error al configurar el puerto */
    }
                  
    DWORD n;
    char enviar;
    int recibido;
    
    
              
/*	enviar = 0xc3;
	                // Se manda
					if(!WriteFile(h, &enviar, 1, &n, NULL))
					{
	                    	
	                }
			  
				             
    /* Para que WaitCommEvent espere el evento RXCHAR */
    SetCommMask(h, EV_RXCHAR);
    while(1) 
	{	
		
    	recibido=0;
        while(1) 
		{
        	ReadFile(h, &recibido, 1/* leemos un byte */, &n, NULL);
            if(!n)
            	break;
            else
			{
            	if(flagcom!=0)
				flagcom++;
				if((recibido==0xAA)&&(flagcom==0))
   				{
        			flagcom=1;
   				}
				if(flagcom==2)
   				{
        			posD = recibido;
        			incD = 127-posD;
				}
   				
   				if(flagcom==3)
   				{
        			posI = recibido;
        			incI = 127-posI;
   				} 
   				if(flagcom==4)
   				{
        			Ax = recibido;
        			Ax = Ax << 8;
   				} 
   				if(flagcom==5)
   				{
				   Ax = Ax + recibido;
   				} 
   				if(flagcom==6)
   				{
        			Gy = recibido;
        			Gy = Gy << 8;
   				} 		
   				if(flagcom==7)
   				{
        			Gy = Gy + recibido;
   				} 	   				
   				if(flagcom==8)
   				{
   				// Leer sensores de seguidor y de obstáculo.
					sensores=recibido;
				// Convertir el valor de los sensores a bits separados.
					sensor_LI=(sensores & 0x04)>>2;
					sensor_LC=(sensores & 0x02)>>1;
					sensor_LD=(sensores & 0x01);
				//Seguidor de línea	
				if(sensor_P == false)
				{
					if(phi<phid)
					{
						if(sensor_LI && !sensor_LD)
							if(sensor_LC)	{KVI=0.8; KVD=1.4;} //0.8, 1.2
							else 			{KVI=0.7; KVD=1.6;}	//0.7, 1.4
							
						else if(!sensor_LI && sensor_LD)
							if(sensor_LC)	{KVI=1.4; KVD=0.8;}
							else 			{KVI=1.6; KVD=0.7;}
								
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
				
					
				// Calulo velocidad derecha.
        			omegaD=incD*esc*iTs;
				// Calculo velocidad izquierda.
        			omegaI=incI*esc*iTs;
        		// Los valores del MPU los paso a valores flotantes con sus respectivas escalas.
        			Xa=-Ax*accel_factor;
        			Yg=Gy*gyro_factor;
        			// Calculo de filtro complementario
        			accelx=Xa*90;//inclinación de -90 a 90 grados de balancin
        			angulox=c1*(angulox_1+Yg*Ts)+cf*accelx;//ecuación del filtro, Yg en grados sobre segundo
        			angulox_1=angulox;//respaldando valor pasado
        			phi=angulox*deg_2_rad;//conversión a rad

					// Calcular el error de angulo
					e_1 = e;
        			e=phid-phi;
        			
        			// Lazo Maestro
        			proporcional=KP*e;
        			derivativa=KD*(e-e_1)*iTs;
					omegad=proporcional+derivativa;
					omegad=omegad+omegaA;
					//omegad=15*sin(6.28*t);
					//omegad=6.28;
					//Saturación de omega_d
					if(omegad > omegaM)
						omegad = omegaM;
					if(omegad < -omegaM)
						omegad = -omegaM;
					
					omegadI = KVI*omegad;
					omegadD = KVD*omegad;
					
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
	                enviar = uRI;
	                // Se manda
					if(!WriteFile(h, &enviar/*puntero al buffer*/, 1/* 1 byte*/, &n, NULL))
					{
	                    	/* Error al enviar */ 
	                }
	                
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
	                enviar = uRD;
	                // Se manda
					if(!WriteFile(h, &enviar/*puntero al buffer*/, 1/* 1 byte*/, &n, NULL))
					{
	                    	/* Error al enviar */ 
	                }

	                //Imprimiendo en pantalla
	                //printf("%i\t\n", pos);
	                //printf("%.2f\t%.2f\t%.2f\n",t,uI,uD);
					//printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\t%d\n",t,uI,uD,omegaI,omegaD,incI,incD);
					//printf("%.2f\t%d\t%d\n",t,Ax,Gy);
	    			//printf("%.3f\t%.3f\t%.3f\t%.3f\n",phid, phi, uI, uD);
	    			printf("%.2f\t%.2f\t%.2f\t%d\t%d\t%d\t%d\n",t,phid,phi,sensor_LI, sensor_LC, sensor_LD, sensor_P);
					/*escribir algunos datos en el archivo*/
	    			fprintf(fp,"%.2f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%i\t%i\t%i\t%i\n",t,phid,phi,omegadI,omegaI,omegadD,omegaD,uI,uD,sensor_LI,sensor_LC,sensor_LD,sensor_P);
	    			t=t+Ts;//t+=Ts;
	    			flagcom=0;
					
				}				
   			}//cierre del else       
            //printf("%c",recibido);
	    	//cout << recibido; /* mostramos en pantalla */
    	}//CIERRE WHILE()
    }//CIERRE WHILE()
fclose(fp);             
return 0;
}


unsigned char v_to_pwm(float voltage)
{
	float pwmf;
	pwmf=escs*voltage;
	pwm=(unsigned char)fabs(pwmf);
	if(pwmf<0)
		pwm=pwm+128;
	return pwm;
}


