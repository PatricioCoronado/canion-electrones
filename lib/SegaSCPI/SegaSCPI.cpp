/*************************************************************************
	DEPARTAMENTO DE ELECTRÓNICA DE SEGAINVEX. UNIVERSIDAD AUTONOMA DE MADRID				
	LIBRERIA PARA ARDUINO SegaSCPI V1
  SISTEMA PARA COMUNICAR UNA COMPUTADORA CON ARDUINO MEDIANTE PUERTO SERIE 
  Fichero de código SegaSCPI.cpp
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
#include "SegaSCPI.h"
/****************************************************************************
	Funciones de PilaCodigoErrires
*****************************************************************************/
/****************************************************************************
 * Inicializador
*****************************************************************************/
void PilaErrorores::begin(uint8_t maxIn)//Constructor
{
		//Crea la pila "arrayErrores" y la apunta con *arrayErrores
		if (maxIndice > MIN_INDICE && maxIndice < MAX_INDICE)
		{

			arrayErrores = new int[maxIn];
			this->maxIndice = maxIn;//Determina la profundidad de la pila
		}
		else
		{
			arrayErrores = new int[MAX_INDICE];
			this->maxIndice = MAX_INDICE;//Determina la profundidad de la pila
		}
		for (this->indice = 0; this->indice < this->maxIndice; this->indice++)
			arrayErrores[this->indice] = 0;//Toda la pila sin error 
		this->indice = 0;
}
/*************************************************************************
   * Error
 **************************************************************************/
int PilaErrorores::error(int codigo)
{
	int retorno;//Valor a devolver -1 o el último cádigo de error
	if (codigo < -1 || codigo >MAX_CODIGO) return 0;
	switch (codigo)
	{
		case -1://Limpia la pila
			for (this->indice = 0; this->indice < this->maxIndice; this->indice++)
				arrayErrores[this->indice] = 0;//Toda la pila sin error 
			indice = 0;
			retorno = -1;//no tiene que devolver un codigo
			break;
		case 0://Devuelve el último y decrementa el índice
			retorno = arrayErrores[this->indice];
			arrayErrores[this->indice] = 0;
			if (this->indice == 0) this->indice = this->maxIndice - 1;//Decrementa indice o pasa de 0 a maxIndice-1
			else this->indice--;
			break;
		default: //Incrementa el índice y añade un error
			if (this->indice == this->maxIndice - 1) this->indice = 0;//Si está en el final de la pila sigue en 0
			else this->indice++;//Aumenta el índice
			arrayErrores[this->indice] = codigo;//Almacena el error
			retorno = -1;//No tiene que devolver un codigo
			break;
	}//Switch
		return retorno;// si 0, no hay errores
}
/****************************************************************************
	Fin de funciones de PilaCodigoErrores
*****************************************************************************/
/**********************************************************************
	Funcion publica: constructor
  1º Inicializa el puntero al nivel Raiz de menús.
  2º Además inicializa el nombre del sistema.
  3º E inicializa la pila de errores de usuario
***********************************************************************/
 SegaSCPI::SegaSCPI(tipoNivel *pRaiz ,const char* nombre,String* errUsuario)
 {
  inicializaSCPI(pRaiz);
  nombreDelSistema=new char[strlen(nombre)+1];
  strcpy(nombreDelSistema,nombre);
  //
  erroresDelUsuario = errUsuario;
 }
/**********************************************************************
 * Hace la inicialización obligatoria:
 * Apunta el puntero del menú raiz 
 * Rellena la lista de códigos de error
 * Crea la pila de errores

**********************************************************************/ 
void SegaSCPI::inicializaSCPI(tipoNivel *pRaiz)
{  
  Raiz=pRaiz; 
  this->codigosError[0]="0 no hay errores";
  this->codigosError[1]="1 Caracter no valido";
  this->codigosError[2]="2 Comando desconocido";
  this->codigosError[3]="3 Cadena demasiado larga";
  this->codigosError[4]="4 Parametro inexistente";
  this->codigosError[5]="5 Formato de parametro no valido";
  this->codigosError[6]="6 Parametro fuera de rango";
  this->pilaErrores.begin(PROFUNDIDAD_PILA_ERR);
}
/**********************************************************************
* Envía el nombre del sistema por el puerto serie
* Solo aquí se utiliza nombreDelSistema   
**********************************************************************/
void SegaSCPI::enviarNombreDelSistema(void)
{
   if(!nombreDelSistema) return; // Si no se asigna nombre sale
   PuertoActual->println(nombreDelSistema);
}
 /*****************************************************************************
	Función pública: 	
  Función de entrada a SegaSCPI. Se la llama con un puntero a tipo Hardwareseria
  que apunta al puerto serial que se está utilizando. Lee el buffer del serial
  en buffCom y lo analiza con "LeeComando" en varias pasadas. Cuando se 
  recibe un comando de un submenu SUBMENU:COMANDO [lista de parametros], en una
	primera pasada del bucle while se procesa SUBMENU: y en otra :COMANDO dejando
  "FinComando" apuntando a la lista de parámetros contenidas en buffCom para
  que el usuario pueda leer los parámetros.
*******************************************************************************/
 void SegaSCPI::scpi(Serial_ * PuertoSerie) 
{
  PuertoActual=PuertoSerie;//Puerto a utilizar
  if (Raiz==NULL)return;//Si no hay menú raiz de comandos salimos
  locCom=0; // Pone a 0 el índice de la cadena por la que recibe del puerto serie
	indComRd = 0; // Pone a cero el indice de la cadena de comando que va a leer
	locCom=PuertoActual->readBytesUntil('\r',buffCom, BUFFCOM_SIZE);
	buffCom[locCom]='\r'; // Añade un terminador porque readBytesUntil se lo quita
	buffCom[locCom+1]='\0'; // Añade un fin de cadena al estilo C
	locCom++;// Ajusta la longitud de la cadena porque le he añadido un caracter más
  while(locCom)// Mientras haya caracteres en el buffer, sigo ejecutando buscando comandos
	{ 
		char serie[LONG_SCPI];//En "serie" se va concatenando los caracteres leidos
		char c; //en "c" se copia el caracter leido desde buffCom a la cadena serie[]
		unsigned char n = 0;//n es el indice de la cadena serie
    do{// Lee caracteres hasta que encuentra un caracter separador 
			c=CaracterBueno(lee_caracter());//verifica si el carácter apuntado en buffCom es valido  
			if (!c){errorscpi(1);return;}//Si no es valido da un error y sale
			serie[n] = c; //Si el caracter es valido lo mete en la cadena "serie"
			if (n < LONG_SCPI - 1){n++;}//si no estamos al final físico de la cadena avanzamos el indice
			else{errorscpi(3);return;} //Si estamos al final de la cadena da un error y sale
		 }while (!separador(c));//Mientras no encontremos un separador seguimos recorriendo la cadena
			if (c != ':') n--; // En caso de ser un serapador distinto de : lo machaco con un 0x00
			serie[n] = 0;// Los : son transmitidos a EjecutaComandos. otros separadores no
      LeeComandos(serie);//Una vez encontrada una cadena valida la envia a ejecutar
	}//while(locCom)
}//scpi(void)
/**************************************************************************
  Función privada: 
  Lee comandos. Se pasa como parametro de entrada una cadena con un comando.
  Si encuentra un comando valido de un submenu, apunta a ese menu y si es
  un comando con puntero a función, la ejecuta. Si no encuentra ni una cosa 
  ni otra, apunta al menú raiz. Y si en este tampoco esta el comando, sale 
  con error. Es una función recursiva.
**************************************************************************/
 void SegaSCPI::LeeComandos(char *cadena)
{
  unsigned char a, n, b/* ,NumComandos*/;
  static /*struct*/ tipoNivel  *PNivel = Raiz, *PNivelSup = Raiz;
   while (*cadena == ' ')cadena++; //Limpiamos los blancos del comienzo
  // Si comienza la cadena con ':' hay que rastrear en el raiz 
  if (cadena[0] == ':') {PNivel = Raiz;   return;}// Apunta al raiz y sale
  n = strcspn(cadena, "? :"); // Busca si existen parametros despues de la funcion y 
  if (n < strlen(cadena))    
	{FinComando = cadena + n;}// si existen apunta FinComando a la cadena que los contiene
	b = PNivel->NumNivInf; // b contiene el numero de comandos en este nivel
  // Compara el comando con cada uno de los que hay en el nivel
  for (a = 0; a < b; a++) {  
    if (
		(n == strlen(PNivel->sub[a].corto) && !strncmp(cadena, PNivel->sub[a].corto, n))
		 ||
        (n == strlen(PNivel->sub[a].largo) && !strncmp(cadena, PNivel->sub[a].largo, n))
	   ) 
		{
		// Si encuentra el comando en el nivel apuntado
		if (PNivel->sub) {//Si hay un nivel debajo...
			PNivelSup = PNivel;//..guarda el nivel actual..        
			PNivel = &(PNivel->sub[a]);//.. y lo apunta bajando un nivel.
		} //Pero si en vez de un nivel inferior hay una función:
		if (PNivel->pf!=NULL) {  // Si hay una funcion a ejecutar...
			PNivel->pf(); // ...la ejecuto. AQUí SE EJECUTA LA FUNCIóN
			PNivel = PNivelSup;// Restauro el puntero un nivel por encima
		}   
      return;// Tanto si ejecuta el comando como si cambia el nivel apuntado, sale
    }//if ((n == strlen(....
  }//for (a = 0; a < b; a++) 
  // Busqueda del comando en el raiz, después de no encontrarlo en el nivel actual
  if (PNivel != Raiz) { // Si el comando no existe en este nivel...
    PNivelSup = PNivel; // ...guarda el nivel actual...
    PNivel = Raiz;// ...y voy al raiz y..
	LeeComandos(cadena);// busco llamando de forma recursiva a "LeeComando"
    if (PNivel == Raiz) PNivel = PNivelSup; // Si tampoco lo encontró
     //en el raiz restauro el nivel
  } //if (PNivel != Raiz)                                   
  else errorscpi(2);     // Si ya estaba en el raiz el comando no existe
}
/***********************************************************************
  Función pública: Gestiona la pila de errores de SegaSCPI
*************************************************************************/
 void SegaSCPI::errorscpi(int numError) 
 {
	int numeroDevuelto;
  if (numError < -1) return;// número bueno es -1 , 0, 1, 2,...
	numeroDevuelto = pilaErrores.error(numError);//Lee el número de error
	if (numeroDevuelto != -1) 
   {
      if(numeroDevuelto<=STRINGS_ERRORES_SCPI-1)
      { //Errores propios del sistema SCPI
        PuertoActual->println(codigosError[numeroDevuelto]);
      }
      else 
      { //Errores de usuario
        if (erroresDelUsuario!=NULL)//Si hay errores de usuario
        //No imprime más de MAX_LONG_STRING_ERR caracteres por seguridad
        PuertoActual->println(erroresDelUsuario[numeroDevuelto-STRINGS_ERRORES_SCPI].substring(0,MAX_LONG_STRING_ERR));
        else PuertoActual->println("error indeterminado");
      }
   }
}
/***********************************************************************
  Función privada: Busca un caracter separador : ; \n \r
************************************************************************/
 unsigned char SegaSCPI::separador(char a) 
 {
  if (a == ':' || a == ';' || a == '\n' || a == '\r') return (1);
  return (0);
}
/**************************************************************************
  Función privada: Busca otros caracteres válidos . ? * - + '
***************************************************************************/
 unsigned char SegaSCPI::valido(char a) 
 {
  if (a == '?' || a == '.' ||  a == '*'  || a == '-' || a == '+' || a == ',' ) return (1);
  return (0);
}
/**************************************************************************
  Función privada: Verifica si el carácter es válido.
   (alfanamerico, separador ó puntuación)
***************************************************************************/
 char SegaSCPI::CaracterBueno(char caracter) 
 {
  if (!isalnum(caracter))       //Si no es alfanumerico...
    if (!isspace(caracter))    // ni un espacio...
      if (!separador(caracter)) // ni un separador admitido...
        if (!valido(caracter)) // ni un caracter especial...
          return (0);// no es un caracter bueno.
  return (toupper(caracter));   //Si es un caracter bueno lo pasa a mayuscula y lo devuelve
}
/***************************************************************************
    Función privada: Lee caracteres del buffer buffCom[]
**************************************************************************/
 char SegaSCPI::lee_caracter()  
 {
  unsigned char carRecv;
  if (locCom == 0) return (0x00); //Si no hay caracteres en buffer salimos con caracter nulo 0
  carRecv = buffCom[indComRd];  //Si lo hay,leemos el "char" correspondiente del buffer
  indComRd++;                   //Apuntamos el indice de lectura al siguiente caracter de buffer
  locCom--;                     // Descontamos contador de "caracteres pendientes" en buffCom
  if (indComRd == BUFFCOM_SIZE) indComRd = 0; // Si se sale del buffCom lo ponemos al pricipio
  return (carRecv);             // Salimos devolviendo el caracter leido
}
/**************************************************************************
	FUNCIONES AUXILIARES PARA LEER LOS PARÁMETROS TRAS EL COMANDO 
  
Estas funciones hay que ejecutarlas dentro de las funciones que responden
a comandos scpi cuando sabemos que hay UN parámetro en el buffer
"FinComando". 

Estas funciones reciben como parámetro principal la dirección de la 
variable que se quiere actualizar. Además de los posibles valores
de la variable para testear el rango.

 Si el carácter FinComando[0] es '?' devuelve el valor actual del parámetro.
  Si es un espacio, lee el parámetro que tiene que estar a continuación, 
  si no está anota un error 4. Si no está en rango anota un error 6. 
 **************************************************************************/
/**************************************************************************
  Función pública.
  Cambia la variable del sistema tipo double cuya dirección se pasa como
  argumento. Además    hay que    pasarle el valor máximo, el mínimo.
  Devuelve 1 si cambió la variable. 0, si   no cambió la variable por
  errores y 2 si devolvió el valor de la variable.
 **************************************************************************/
 int SegaSCPI::actualizaVarDecimal(double *pVariableDelSistema,double Maximo, double Minimo)
{
  double Var; //variable intermedia
  // Si solo pregunta por el dato, se responde y sale
  if(FinComando[0]=='?')  
  {
	  //Serial.println(*pVariableDelSistema);
     PuertoActual->println(*pVariableDelSistema); 
    return 2;
  }      
  if(FinComando[0]==' ')// Si el primer carácter de FinComado es 'espacio' lee el parámetro
  {   
    Var=strtod(FinComando,NULL);//Lee un double de la cadena FinComando
  }   
  else {errorscpi(4);return 0;} // Si FinComando no empieza por 'espacio' Error!Parómetros inexistentes.
// Si el número de parámetros leidos es correcto comprueba los rangos
  if ((Var <= Maximo && Var >= Minimo)){*pVariableDelSistema=Var;return 1;}//si esta en rango,cambia la variable
  else  {errorscpi(6);return 0;}// si no está en rango da el error correspondiente y sale
}
/**************************************************************************
   Función pública.
   Cambia la variable del sistema tipo int cuya dirección se pasa como argumento.
    Además hay que pasarle el valor máximo y el mínimo.
   Devuelve 1 si cambió la variable. 0, si no cambió la variable por errores.
   Y 2 si devolviá el valor de la variable.
 **************************************************************************/
 int SegaSCPI::actualizaVarEntera(int *pVariableDelSistema,int Maximo, int Minimo)
{
  unsigned int np; // námero de parámetros leido por sscanf
  int Var; //variable intermedia
  // Si solo pregunta por el dato, se responde y sale
  if(FinComando[0]=='?')  
	{
	  //Serial.println(*pVariableDelSistema);
     PuertoActual->println(*pVariableDelSistema); 
    return 2;
  }      
   // Si el primer carácter de FinComado es 'espacio' lee los parámetros
    if(FinComando[0]==' ')  
  {   
    np = sscanf(FinComando,"%u",&Var);// Lee la cadena de parámetros
    if(np != 1){errorscpi(5);return 0;}// Si no lee 1 parámetro Error!:Formato de parámetro no válido
  }   
  else {errorscpi(4);return 0;} // Si el comando no empieza por 'espacio'Error!Parámetros inexistentes.
// Si el número de parámetros leidos es correcto comprueba los rangos
  if ((Var<=Maximo && Var>=Minimo)){*pVariableDelSistema=Var;return 1;} //si esta en rango, cambia la variable
  else  errorscpi(6);
  {
    return 0;
  }// si no está en rango da el error "parámetro fuera de rango"
}
/**************************************************************************
 Función pública.
 Cambia una variable tipo int que puede tener un conjunto discreto de valores
 Se pasa como argumento un array con los posibles valores y un entero con 
 la cantidad de posibles valores.
 Devuelve 1 si cambió la variable. 0, si no cambió la variable por errores.
 Y 2 si devolvió el valor de la variable.
 **************************************************************************/
 int SegaSCPI::actualizaVarDiscreta(int *pVariableDelSistema,int ValoresPosibles[],int NumeroValores)// 
{
  unsigned int np; // número de parámetros leido por sscanf
  int i, Var; //variable intermedia
  // Si solo pregunta por el dato, se responde y sale
  if(FinComando[0]=='?')  
  {
    //Serial.println(*pVariableDelSistema);
     PuertoActual->println(*pVariableDelSistema); 
    return 2;
  }      
   // Si el primer carácter de FinComado es 'espacio' lee los parámetros
    if(FinComando[0]==' ')  
  {   
    np = sscanf(FinComando,"%u",&Var);// Lee el nuevo valor
    if(np != 1){errorscpi(5);return 0;}// Si no lee 1 parámetro Error!:Formato de parámetro no válido
  }   
  else {errorscpi(4);return 0;} // Si el comando no empieza por 'espacio'Error!Parámetro inexistente.
// Si se ha leido un parámetro se testea su valor
  for(i=0;i<NumeroValores;i++)
  {
  if (Var == ValoresPosibles[i]) // Si encuentra una coincidencia cambia el valor y sale
  {*pVariableDelSistema=Var;return 1;}
  }
  errorscpi(6);// si no está en la lista de valores posible da el error "parámetro fuera de rango"
  return 0;
}
/**************************************************************************
   Función pública.
   Cambia la variable del sistema tipo booleana cuya dirección se pasa 
   como argumento
 **************************************************************************/
 int SegaSCPI::actualizaVarBooleana(bool *pVariableDelSistema)
{
  unsigned int np; // número de parámetros leido por sscanf
  int Var; //variable intermedia
  // Si solo pregunta por el dato, se responde y sale
  if(FinComando[0]=='?')  
	  {
       //Serial.println(*pVariableDelSistema);
        PuertoActual->println(*pVariableDelSistema);  
       return 2;
	  }      
   // Si el primer carácter de FinComado es 'espacio' lee los parámetros
    if(FinComando[0]==' ')  
  {   
    np = sscanf(FinComando,"%u",&Var);// Lee la cadena de parámetros
    if(np != 1){errorscpi(5);return 0;}// Si no lee 1 parámetro Error!:Formato de parámetro no válido
  }   
  else {errorscpi(4);return 0;} // Si el comando no empieza por 'espacio'Error!Parámetros inexistentes.
// Si el número de parámetros leidos es correcto comprueba los rangos
  if ((Var == 0 || Var == 1)){*pVariableDelSistema=Var;return 1;} // si vale 0  1, cambia_la variable
  else  {errorscpi(6); return 0;}// si no está en rango da el error de "parámetro fuera de rango"
} 
/******************************* FIN **************************************/
#define DEBUG  