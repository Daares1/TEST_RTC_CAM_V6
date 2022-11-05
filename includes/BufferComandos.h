#ifndef __MEMORIA_H__
#define __MEMORIA_H__

#include <stm32f4xx.h>			

#define TAMANO_MAX_FIFO		64
#define FULL_75 49
#define FULL_25 16
#define OK 1
#define NOK 0					



typedef struct  
 {						
   char contenido;
   char ack;
   	
 }var_datos;

extern var_datos datos;



char escribir_fifo(char el_datico);

var_datos leer_cola(void);

#endif
