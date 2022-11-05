#include "BufferComandos.h"

var_datos datos;
var_datos arreglo_datos[TAMANO_MAX_FIFO];

char contador = 0;	   //Memoria Ocupada
static char ref_escribir;//Referencia de posicion a escribir
static char ref_leer;	 //Referencia de posicion a leer

char escribir_fifo(char el_datico)
{
    var_datos aux_dato;
	aux_dato.contenido=el_datico;
	aux_dato.ack=1;

	arreglo_datos[ref_escribir]=aux_dato;
	ref_escribir++;	
	contador++;

	if(ref_escribir == TAMANO_MAX_FIFO)
	{
		ref_escribir = 0;
	}

   return contador;

} //void escribir_cola(var_control_cola *ptr_cola)


 
var_datos leer_cola()
{
   var_datos el_datico;
	if((contador) > 0)
	{   
	   
	    (el_datico.contenido) = arreglo_datos[ref_leer].contenido;
	    (el_datico.ack) = arreglo_datos[ref_leer].contenido;
		ref_leer++;
	    contador--;
		
	

		if(ref_leer == TAMANO_MAX_FIFO)
		{
			ref_leer = 0;
		}

	  	
	}/*if(buffer > TAMANO_MAX_BUFFER)*/
	else
	{
		(el_datico.contenido) = 0;
	    (el_datico.ack) = 0;
	}
	return el_datico;
	
}//void leer_cola(var_ctr_cola *ptr_cola) 
