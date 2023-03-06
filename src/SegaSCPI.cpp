/*********************************************************************
 SegaSCPI
 Uso de la librería SegaSCPI para  comunicar
 un PC con Arduino a través del puerto serie "Serial" 
**********************************************************************/
#include <Arduino.h>
#include "SegaSCPI.h"

extern void funcion_test_externa(int);

#define puertoSerie segaScpi.PuertoActual->println
/**********************************************************************
Prototipos de funciones
***********************************************************************/
//Funciones scpi comunes a todos los sistemas
void errorSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void clsSCPI(void);
// Funciones particulares de nuestro sistema
void funcion_test_local(void);//Función del menú raiz
/*
	para provocar un reset del watchdog
*/
/**********************************************************************
 Menú principal que contiene comandos
**********************************************************************/
MENU_SCPI  //menú de  comandos y submenús OBLIGATORIO
{
	SCPI_COMANDO(WATCHDOG,WD,funcion_test_local)   //
	
	// TO DO Añadir aquí comandos o submenús
	//Comandos de funciones propias de SegaSCPI 
	SCPI_COMANDO(ERROR,ERR,errorSCPI)// Envía el ultimo error
  	SCPI_COMANDO(*IDN,*IDN,idnSCPI)// Identifica el instrumento
	SCPI_COMANDO(*OPC,*OPC,opcSCPI)// Devuelve un 1 al PC
	SCPI_COMANDO(*CLS,*CLS,clsSCPI)// Borra la pila de errores
};
//declaramos el nivel Raíz cuya dirección se pasa a la función begin de SegaSCPI
tipoNivel Raiz[]= SCPI_RAIZ //// Declaración OBLIGATORIA del nivel Raiz. Siempre igual
//Opcionalmente podemos definir la lista de errores (a partir de 7):
/**********************************************************************
 		errores del sistema definidos por el usuario
**********************************************************************/

String misErrores[]=
{//Los errores de 0 a 6 son de scpi
	"7 la variable 1 no se ha cambiado",
	"8 la variable 2 no se ha cambiado",
	"9 otro error",
	"10 y otro error",

};
//Ahora el código habitual de Arduino:
SegaSCPI segaScpi(Raiz,"LEG 24 Electron Gun",misErrores);//Instanciamos el objeto SCPI	global
/****************************************************************
	
*****************************************************************/
void funcion_test_local(void)
{
	delay(2000);
}
/****************************************************************
*****************************************************************/

/****************************************************************
	
*****************************************************************/

/************************************************************************
    Funciones scpi comunes a todos los sistemas
 *************************************************************************/
 /************************************************************************
    Función del Comando: ERROR ó ERR
    Envia por el puerto el último error registrado por SEGAINVEX-SCPI
 *************************************************************************/
void errorSCPI(void){segaScpi.errorscpi(0);}
/*************************************************************************
  Función del Comando: *IDN"
   Envia por el puerto una cadena que identifica al sistema
 *************************************************************************/
void idnSCPI(void){segaScpi.enviarNombreDelSistema();}
 /************************************************************************
  Función del Comando:*OPC
  Envia por el puerto el carácter uno
 *************************************************************************/
void opcSCPI(void){	segaScpi.PuertoActual->println("1");}
/*************************************************************************
    Comando: CLS
    Limpia la pila de errores de SEGAINVEX-SCPI
 *************************************************************************/
void clsSCPI(void){segaScpi.errorscpi(-1);}
/*************************************************************************/
/*******************************FIN***************************************/
