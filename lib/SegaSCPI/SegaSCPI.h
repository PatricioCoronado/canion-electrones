/*************************************************************************
	DEPARTAMENTO DE ELECTRÓNICA DE SEGAINVEX. UNIVERSIDAD AUTONOMA DE MADRID				
	LIBRERIA PARA ARDUINO SegaSCPI V1
  SISTEMA PARA COMUNICAR UNA COMPUTADORA CON ARDUINO MEDIANTE PUERTO SERIE 
  Fichero de cabecera SegaSCPI.h
**************************************************************************/
/*
	Copyright © 2017 Mariano Cuenca, Patricio Coronado
	
	This file is part of SegaSCPI
    SegaSCPI is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    SegaSCPI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with SegaSCPI.  If not, see <http://www.gnu.org/licenses/>.
**************************************************************************/
/************************************************************************* 
		Mejoras futuras:
		1)Poner el menú de SegaSCPgit I en flash para no consumir RAM
 *************************************************************************/
 /************************************************************************* 
		Funciona con todos los puertos Serial tipo HardwareSeria no con SerialUSB.
		
		Para enviar datos por el Serial por el que llegó el comando SCPI
    desde el programa principal utiliza la el puerto de la clase:
    segaScpi SegaSCPI;//Instanciación del objeto
    segaScpi.PuertoActual->println(segaScpi.nombreSistema);
 *************************************************************************/
#ifndef SegaSCPI_H
#define SegaSCPI_H
//
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
/*************************************************************************
                CONSTANTES
*************************************************************************/
#define BUFFCOM_SIZE 32 //Longitud del buffer de lectura del comando recibido
#define LONG_SCPI 32 // Longitud máxima del comando sin contar parámetros
#define MAX_LONG_STRING_ERR 64 //Máxima longitud de string de error
#define MIN_INDICE 4 //Tamaño mínimo de la pila 
#define MAX_INDICE 16//Tamaño máximo de la pila y Valor por defecto
#define MAX_CODIGO 255//Los códigos de error posibles son de 1 a MAX_CODIGO
#define PROFUNDIDAD_PILA_ERR 16 //Tamaño de la pila de errores
#define STRINGS_ERRORES_SCPI 7 //Número de strings de errores de scpi
/*************************************************************************
                TIPOS
*************************************************************************/
struct tipoNivel
{             
  int   NumNivInf;// Número de niveles por debajo de este
  const char *largo, *corto;// Nombre largo y nombre corto del comando
  void (*pf)();// Puntero a función a ejecutar
  tipoNivel *sub;// Puntero a la estructura de niveles inferiores
};

/****************************************************************************
CLASE PilaErrorores: Monta una pila circular de enteros
para guardar números de errores. La pila puede tener una profundidad
de MIN_INDICE a MAX_INDICE. los números de error que guarda van
de 1 a 255. Para crear un objeto hay que introducir un entero
de de MIN_INDICE a MAX_INDICE si no crea una pila de MAX_INDICE.
Para leer o enviar errores a la pila hay que llamar al método
"error" con un parámetro int entre -1 y MAX_CODIGO. Si es -1
resetea la pila, si es cero devuelve el último número de error,
si el número devuelto es 0 es que no hay errores.
Si el número introducido es de 1 a MAX_CODIGO lo guarda en la pila.
La pila es circular y si no se leen los números de error, cuando
se supere la profundidad de la pila los números de error antiguos
se pisan por los nuevos.
*****************************************************************************/
class PilaErrorores //Pila de codigo de errores
{
  #define MIN_INDICE 4 //Tamaño mínimo de la pila 
  #define MAX_INDICE 16//Tamaño máximo de la pila y Valor por defecto
  #define MAX_CODIGO 255//Los número de error posibles son de 1 a MAX_CODIGO
	//private:
  public:
	  uint8_t maxIndice;//Profundidad de la pila de errores
	  uint8_t indice;// Indice de la pila
	  int *arrayErrores;//array de números de error
    void begin(uint8_t maxIndice);//Constructor
    int error(int);
 };// Fin de la clase PilaErrores
  /***********************************************************************/
/*************************************************************************
                CLASE SEGA SCPI
**************************************************************************/ 
class SegaSCPI
{
public:
    //Variables y objetos públicas
      char *FinComando;// Puntero al final del comando para leer parámetros
      Serial_ *PuertoActual;
    //Métodos públicos
      SegaSCPI(tipoNivel *pRaiz ,const char*,String*); //Constructor
      void scpi(Serial_* );//Función de entrada a SegaSCPI
      void errorscpi(int); //Gestióna la pila de números de error 
      int actualizaVarEntera(int *,int,int);//Actualiza variable entero
      int actualizaVarDiscreta(int *,int*,int);//Actualiza entero discreta
      int actualizaVarBooleana(bool *);//Actualiza Booleano
      int actualizaVarDecimal(double *,double,double);//Actualiza decimal
      void enviarNombreDelSistema(void);
private://Variables privadas  
      char * nombreDelSistema;
      int lonPila=PROFUNDIDAD_PILA_ERR;//Tamaño de la pila de números de error
      PilaErrorores pilaErrores;//Pila de números de error
      String codigosError[STRINGS_ERRORES_SCPI];//Array de strings con la explicación de los errores de scpi
      String *erroresDelUsuario;//puntero a array de strings con la explicación de los errores del usuario
      int PuertoSCPI;
      tipoNivel *Raiz;
      //String *codigosError;//Array de cadenas con la descripción de los errores
      char buffCom[BUFFCOM_SIZE]; // Buffer leido del puerto serie con el comando
      int locCom; // Cantidad de caracteres en el buffer
      unsigned char indComRd;// = 0;
    //Métodos privados
      void inicializaSCPI(tipoNivel*);
      char lee_caracter(void);
      unsigned char separador(char);
      unsigned char valido(char);
      char CaracterBueno(char);
      void LeeComandos(char *cadena);
};
/***********************************************************************
			        macros para definir menús
************************************************************************/      
// Para definir submenús 
#define SCPI_SUBMENU(X,Y) sizeof(X)/sizeof(*X), #X,#Y,NULL,X,  
// Para definir comandos
#define SCPI_COMANDO(X,Y,Z) 0, #X,#Y,Z,NULL, //Para definir comandos 
//Para definir el nivel raiz de comandos
#define SCPI_RAIZ {sizeof(NivelDos)/sizeof(*NivelDos),"","",NULL,NivelDos}; 
#define MENU_SCPI tipoNivel NivelDos[]= 
/************************************************************************/
#endif 