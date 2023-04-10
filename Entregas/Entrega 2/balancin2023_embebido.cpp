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

//Variables para recibir datos
unsigned char flagcom=0,flagfile=0; //de 8 bits
signed char temp=0;
float t=0,phid=-0.1485,phi=0,omegadI=0,omegaI=0,omegadD=0,omegaD=0,uI=0,uD=0;
int recibido=0;

// Sensores
unsigned char sensores;
bool sensor_LI = false;
bool sensor_LD = false;
bool sensor_LC = false;
bool sensor_P  = false;

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
        			temp = recibido;
        			phi = (0.0078431373 * temp) + 0.0039215686;
				}
   				if(flagcom==3)
   				{
        			temp = recibido;
        			omegadI = (0.3137 * temp) + 0.1569;
   				} 
   				if(flagcom==4)
   				{
        			temp = recibido;
        			omegaI = (0.3137 * temp) + 0.1569;
   				} 
   				if(flagcom==5)
   				{
				   	temp = recibido;
        			omegadD = (0.3137 * temp) + 0.1569;
   				} 
   				if(flagcom==6)
   				{
        			temp = recibido;
        			omegaD = (0.3137 * temp) + 0.1569;
   				} 		
   				if(flagcom==7)
   				{
        			temp = recibido;
        			uI = (0.1176 * temp) + 0.0588;
   				} 
				if(flagcom==8)
   				{
        			temp = recibido;
        			uD = (0.1176 * temp) + 0.0588;
   				} 	   				
   				if(flagcom==9)
   				{
   					// Leer sensores de seguidor y de obstáculo.
					sensores=recibido;
					// Convertir el valor de los sensores a bits separados.
					sensor_P =(sensores & 0x08)>>3;
					sensor_LI=(sensores & 0x04)>>2;
					sensor_LC=(sensores & 0x02)>>1;
					sensor_LD=(sensores & 0x01);
					
					if (sensor_P == true)
						phid = -0.430;
					
					//Imprimir en monitor
					printf("%.2f\t%.3f\t%.3f\n",t,phid,phi);
					
					//Guardar en archivo
	    			fprintf(fp,"%.2f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%i\t%i\t%i\t%i\n",t,phid,phi,omegadI,omegaI,omegadD,omegaD,uI,uD,sensor_LI,sensor_LC,sensor_LD,sensor_P);
	    			t=t+Ts;
	    			flagcom=0;
				}
								
   			}//cierre del else       
    	}//CIERRE WHILE()
    }//CIERRE WHILE()
	fclose(fp);             
	return 0;
}

