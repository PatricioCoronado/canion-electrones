/*********************************************************************
 Test_SegaSCPI.cpp
 Ejemplo de uso de la librería SegaSCPI para  comunicar
 un PC con Arduino a través del puerto serie "Serial" 
**********************************************************************/
#include <Arduino.h>
# include "SegaSCPI.h"
// LA librería debe estar aquí o cambiar el path al de la librería
//# include "C:\Users\PACOCO\Mis Proyectos\SegaSCPI\SegaSCPI.h"
//# include "C:\Users\PACOCO\Mis Proyectos\SegaSCPI\SegaSCPI.cpp"
/**********************************************************************
Prototipos de funciones
***********************************************************************/
//Funciones scpi comunes a todos los sistemas
void errorSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void clsSCPI(void);
// Funciones particulares de nuestro sistema
void funcion1(void);//Función del menú raiz
void funcion2(void);//Función del menú raiz
void funcion11(void);//Función del submenú SUBMENU1
void funcion12(void);//Función del submenú SUBMENU1
/**********************************************************************
 Submenú al que llamamos por ejemplo "SUBMENU1". Contiene dos 
 comandos, a los que llamamos,COMANDO11 con nombre
 abreviado C11 y COMANDO12 con nombre abreviado C12. El 
 primero hará que se ejecute la función funcionC11 y el 
 segundo la función funcionC12() que definiremos como 
 cualquier otra función.
**********************************************************************/
 tipoNivel SUBMENU1[] = //menú de comandos OPCIONAL
{
	SCPI_COMANDO(COMANDO11,C11,funcion11)//Comando que ejecuta la función funcion11
	SCPI_COMANDO(COMANDO12,C12,funcion12)//Comando que ejecuta la función funcion12
	// TO DO Añadir aquí más comandos o submenús
};
/**********************************************************************
 Menú principal que contiene comandos y submenús
 Tiene dos comandos:
 	COMANDO1 abreviado C1 que ejecuta la función funcion1
	COMANDO2 abreviado C2 que ejecuta la función funcion2
  	El submenú definido más arriba.	SUBMENU1 abreviado SM1 
	Y 4 comandos particulares de SegaSCPI
**********************************************************************/
MENU_SCPI  //menú de  comandos y submenús OBLIGATORIO
{
	SCPI_SUBMENU(SUBMENU1,SM1 )	//Submenú con comandos declarado más arriba
	SCPI_COMANDO(COMANDO1,C1,funcion1) //Comando que ejecuta la funcion void funcion1
	SCPI_COMANDO(COMANDO2,C2,funcion2) //Comando que ejecuta la función funcion2
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
SegaSCPI segaScpi(Raiz,"Nombre de mi sistema",misErrores);//Instanciamos el objeto SCPI	global
/**********************************************************************
 					setup
**********************************************************************/
void setup() 
{
	//segaScpi.begin(Raiz, &nombreSistema,misErrores);//Inicializamos 
	// Abre el puerto serie
	Serial.begin(115200); 
	Serial1.begin(115200);
		// TO DO Poner aquí el código de usuario 
}
/**********************************************************************
 					loop
**********************************************************************/
void loop()
{
  // Si recibe algo por el puerto Serial lo procesa con SegaSCPI 
  if (Serial.available()){segaScpi.scpi(&Serial);}
  // Si recibe algo por el puerto Serial1 lo procesa con SegaSCPI 
  if (Serial1.available()){segaScpi.scpi(&Serial1);}

	// TO DO Poner aquí el código de usuario

}

//Ahora solo nos queda definir las funciones de nuestro sistema:

/****************************************************************
	Comando COMANDO1 ó C1
	Ejecuta la función funcion1();
*****************************************************************/
void funcion1(void)
{
 segaScpi.PuertoActual->println
 ("Se ha recibido el COMANDO1 y se ha ejecutado la funcion funcion1");	
// Ponemos unos errores en la pila para pedirlos con el comando err?
	segaScpi.errorscpi(1);
	segaScpi.errorscpi(2);
 	segaScpi.errorscpi(3);
	segaScpi.errorscpi(4);
	segaScpi.errorscpi(5);
	segaScpi.errorscpi(6);
	segaScpi.errorscpi(7);
	segaScpi.errorscpi(8);
	segaScpi.errorscpi(9);
	segaScpi.errorscpi(10);
}
/****************************************************************
	ComandoCOMANDO2 ó C2
	Ejecuta la función funcion12();
*****************************************************************/
void funcion2(void)
{
 segaScpi.PuertoActual->println
 ("Se ha recibido el COMANDO2 y se ha ejecutado la funcion funcion2");	
}
/****************************************************************
	Comando SUBMENU1:COMANDO11 ó SM1:C11
	Ejecuta la función funcion11();
	Cambia el valor de la variable tipo double "Variable1"
*****************************************************************/
void funcion11(void)
{
	static double Variable1=1.1;
	int Resultado;
	Resultado=segaScpi.actualizaVarDecimal(&Variable1,10.0,0.0);
	switch (Resultado)
	{
		case 0:
			segaScpi.PuertoActual->println("No Se cambio el valor de la Variable1");
			segaScpi.errorscpi(7);
		break;
		case 1:
			segaScpi.PuertoActual->println("Se cambio el valor de la Variable1");
		break;
		case 2:
			segaScpi.PuertoActual->println("Se envio al PC el valor de la Variable1");
		break;
	}
}
/*****************************************************************
	Comando SUBMENU1:COMANDO11 ó SM1:C12
	Ejecuta la función funcion12();
	Cambia el valor de la variable discreta tipo int "Variable2"
	que solo puede valer 1, 10 o 100
*****************************************************************/
void funcion12(void)
{
	int Resultado;
	static int Variable2=100;
	int ValoresVariable2[]={1,10,100};
	Resultado=segaScpi.actualizaVarDiscreta(&Variable2,	ValoresVariable2,sizeof(ValoresVariable2));
	switch (Resultado)
	{
		case 0:
		segaScpi.PuertoActual->println("No Se cambio el valor de la Variable2");
		segaScpi.errorscpi(8);
		break;
		case 1:
			segaScpi.PuertoActual->println("Se cambio el valor de la Variable2");
		break;
		case 2:
			segaScpi.PuertoActual->println("Se envio al PC el valor de la Variable2");
		break;
	}	
}
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
/*
  Prueba a enviar por el puerto serie los comandos (terminados con \r)
  sm1:c12?
  sm1:c12 10
  sm1:c12?
  c1
  c2
  err?
  *idn
  err?:*idn
*/
/*******************************FIN***************************************/