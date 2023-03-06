/**************************************************************
 * UNIVERSIDAD AUTÓNOMA DE MADRID
 *  Proyecto: 20211069
 *  Unidad Electronica para cañón de Electrones LEED
 *  Autor: Patricio Coronado Collado
 *  Versión: 1.0 febrero 2023
 *  Aplicación para SAMD21 ecosistema Arduino Seeeduino XIAO.
 *  Descripción: Lee 4 valores del equipo en el que se monta;
 *  La corriente del filamento, la tensión "energy la tensión
 *  mesh retard  corriente "energy". Además lee el set point
 *  de corriente de filamento de un rotary encoder. La consigna
 *  de corriente de filamento la establece una salida PWM 
 *  entre 10 y 990, con 10 la corriente de filamento es de
 *  3,1A y con 990 es de menos de 10mA.
 *  Ultima actualizacion 06-03-2022
 **************************************************************/
#include <Arduino.h>
// Recursos utilizados
#include <U8g2lib.h>          //Display
#include <Wire.h>             //I2C para el display y el ADC
#include <TimerTCC0.h>        //Timer para periodo de muestreo
#include <Adafruit_ADS1015.h> //ADC ADS1115
#include "WDTZero.h"          //Watchdog
#include <FlashStorage.h>     //Para emular EEPROM
#include "SegaSCPI.h"
/*************************************************************
  Macros
**************************************************************/
#define DEPURACION(x) if(depuracion) Serial.println(x)
/**************************************************************
 * recursos opcionales
 * ************************************************************/
#define PUERTO_SERIE
//#define EEPROM //Sería para guardar el dutycicle pero no lo usaré
#define WATCHDOG //Resetea el sistema si se cuelga
/**************************************************************
 * Constantes y variables primitivas
 ***************************************************************/
bool depuracion = false; //a true para depurar, false para montar en el equipo
// Ganancias para calcular los valores reales de las medidas
//V Energy 
#define PENDIENTE_VE 0.0075     // Ver el excel "Ajuste curvas del DAC.xlsx"
#define b_VE 0.1335
//Mesh retard
//#define PENDIENTE_MR 0.00619     // Ver el excel "Ajuste curvas del DAC.xlsx"
//#define B_MR 0.7168
#define PENDIENTE_MR 0.006179     // Ver el excel "Ajuste curvas del DAC.xlsx"
#define B_MR 0.15571
// I Energy
#define GAIN_IENERGY 10.8     // para ver la corriente en uA
//Filamento
#define GAIN_IFILAMENT 1.54 // Ganancia de la lectura del filamento
// Pines del rotary encoder (poner condensador 100nF a masa) y variables para el duty cicle
#define CLK_ROTARY_ENCODER 6
#define DT_ROTARY_ENCODER 7
short dutyCicle = 990;               // DT del PWM, consigna de corriente de filamento-. Al mínimo
short dutyCicleAnterior=0;
short dutyCicleEnFlash=0;
#ifdef EEPROM
  FlashStorage(dutyCicleBackUp, short);      // Reserva memoria en la flash emulada para el duty cicle 
#endif
boolean cambiaDutyCicle=false;//Flag para saber si hay que actualizar el dutycicle en el loop
//
#define LECTURAS_ADC 20            // LECTURAS_ADC para hacer medias
short contadorLecturasADC;         // Contador de muestras acumuladas
#define FREQ_PWM 200               // Frecuencia PWM 200Hz. Armónico de la frecuencia de muestreo para minimizar ruido en las medidas
#define TS_20ms 20000 /*microsegundos*/ // Periodo de muestreo, 20ms frecuencia de red para minimizar ruido
//#define TS_40ms 40000 /*microsegundos*/ // Periodo de muestreo, 40ms
#define PIN_PWM 10                 // Pin de salida PWM
#define PIN_TEST0 0                // Pin para test
#define PIN_TEST1 1                // Pin para test
#define PIN_TEST2 2                // Pin para test
#define PIN_TEST3 3                // Pin para test
#define ENABLE_FILAMENTO 9            // Pin para habilitar el filamento
#define LSB_ADS 62.50191F / 1000000.0 // LSB del AD1115 con VFS=2.048V y 15 bits (el bit 16 es el de signo)
long adc0 = 0;                        // lectura del ADC0 acumuladas
long adc1 = 0;                        // lectura del ADC1 acumuladas
long adc2 = 0;                        // lectura del ADC2 acumuladas
long adc3 = 0;                        // lectura del ADC3 acumuladas
uint16_t _adc0 = 0; // lectura del ADC0 simples
uint16_t _adc1 = 0; // lectura del ADC1 simples
uint16_t _adc2 = 0; // lectura del ADC2 simples
uint16_t _adc3 = 0; // lectura del ADC2 simples
float a0;//Medidas en números reales
float a1;
float a2;
float a3;
bool leerDAC=false;//Para indicar que hay que leer el ADC
short contadorLoops=0;
/**************************************************************
 *  Objetos
 ***************************************************************/
extern SegaSCPI segaScpi;
Adafruit_ADS1115 ads; // Convertidor ADC I2C: ADS1115
// U8X8_SH1106_128X64_NONAME_HW_I2C display(RESET /*pin de reset*/); // Display, reset por pin
U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE); // Display //Reset por red R-C (10Koh,10uF) OK
//   U8X8_SH1106_128X64_NONAME_HW_I2C display(RESET_DISPLAY); // Display //Reset pin 0 que finalmente no se utilizó, ya que se alimentó a 5V
// Watchdog
#ifdef WATCHDOG
  WDTZero WatchDog; // Objeto Watchdog
#endif  
/**************************************************************
 * funciones
 ***************************************************************/
void encoder(void);                   // Interrupción de los pines del rotary encoder
void timerTS(void);                   // Interrupción del timer de captura de datos
void display_saludo(void);            // Presentación inicial UAM / SEGAINVEX
#ifdef WATCHDOG
  void WatchDog_reset(void);
#endif  
/*************************************************************
  Display
**************************************************************/
void display_saludo(void)//Cartel de presentación inicial
{
  display.setFont(u8x8_font_8x13_1x2_f);
  display.clear();
  display.drawString(0, 0, "     UAM      ");
  display.drawString(0, 2, "  SEGAINVEX");
  display.drawString(0, 4, " ELECTRONICA");
  display.drawString(0, 6, "LEG24 ELECT.GUN");
}
/**************************************************************
 * Rutina de la interrupción del encoder 4.6us
 * attachInterrupt(CLK_ROTARY_ENCODER, encoder, FALLING);
 * ***********************************************************/
void encoder(void)
{
  //digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL)); // Led para ver la latencia
  digitalWrite(PIN_LED, !digitalRead(PIN_LED)); // Led para ver la latencia
  digitalWrite(PIN_TEST3, LOW);
  boolean clk = digitalRead(CLK_ROTARY_ENCODER); //Lee el pin que provoca la interrupción
  if (clk) return;//Si no está a nivel bajo sale sin hacer nada
  //Si clk está bajo, continua
  boolean dt = digitalRead(DT_ROTARY_ENCODER);
  short _dutyCicle=dutyCicle; //Guarda el valor actual de dutyCicle de entrada
  //if (dt == false) //Poner la que registre el sentido correcto
  if (dt == true)
  {
    dutyCicle += 10;
  }
  //if (dt == true)
  if (dt == false)
  {
    dutyCicle -= 10;
  }

  if (dutyCicle < 10)
    dutyCicle = 10;
  if (dutyCicle > 990)
    dutyCicle = 990;
  
  if(_dutyCicle!=dutyCicle)//Si hay que cambiar el dutyCicle levanta el flag para que "loop" lo cambie
    
  {
    cambiaDutyCicle=true;
  }
  digitalWrite(PIN_TEST3, HIGH);
}
/**************************************************************
 * lectura de los ADCs (4mS)
 * ***********************************************************/
void timerTS(void)
{
  leerDAC=true;
}
/**************************************************************
 * Función que se ejecuta cuando actua el watchdog
 * para que no actue el watchdog hay que resetear 
 * su contador en el loop así "WatchDog.clear();"
 * NO SE EJECUTA ¿?
 * ***********************************************************/
#ifdef WATCHDOG
  void WatchDog_reset(void)
  {
    digitalWrite(ENABLE_FILAMENTO, LOW);//Apaga el filamento de urgencia
    #ifdef EEPROM
      dutyCicleBackUp.write(dutyCicle);      // Guarda el dutycicle en Flash
    #endif
    Wire.flush();//Por si se cuega por el wire
    Wire.begin();
    //Por si es un problema del display
    display.initDisplay();//Esto hace un reset hardware...
    display.begin();//Después del reset hardware necesita un begin
  }
#endif
/**************************************************************
 * setup
 ***************************************************************/
void setup(void)
{
  //Pines
  pinMode(ENABLE_FILAMENTO, OUTPUT);
  digitalWrite(ENABLE_FILAMENTO, LOW);// Lo primero inhabilita el filamento
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Apaga el led para no consumir. Se apaga con high
  digitalWrite(PIN_LED_RXL, HIGH); //
  digitalWrite(PIN_LED_TXL, HIGH);//led TXL
  pinMode(CLK_ROTARY_ENCODER, INPUT);//Pullup externo
  pinMode(DT_ROTARY_ENCODER, INPUT);//Pullup externo
  pinMode(PIN_TEST0, OUTPUT);
  pinMode(PIN_TEST1, OUTPUT);
  pinMode(PIN_TEST2, OUTPUT);
  pinMode(PIN_TEST3, OUTPUT);
  digitalWrite(PIN_TEST0, HIGH);
  digitalWrite(PIN_TEST1, HIGH);
  digitalWrite(PIN_TEST2, HIGH);
  digitalWrite(PIN_TEST3, HIGH);
  #ifdef PUERTO_SERIE
      Serial.begin(115200);//
  #endif      
  display.begin();// Display
  display.setBusClock(300000);//ADS1115 puede llegar a 400KHz
  Wire.setTimeout(5/*milisegundos*/); // Tieme out del I2C, para salir de cuelgues por el i2c
  // Convertidor AD
  ads.begin(); // Inicializa conversor ADS1115
  // ads.setGain(GAIN_TWOTHIRDS);
  ads.setGain(GAIN_TWO);
  //Para utilizar el convertidor ADC interno
  //analogReference(AR_INTERNAL2V2); //Referencia
  /*
  Según la ganancia que programemos tendremos un VFS y un LSB
  teniendo en cuenta que son 15 bits de resolución ya que el
  primer bit es el de signo. LSB=VFS/2^15
  GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,VFS=6,144V 15 BITS
  GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,VFS=4,096V 15 BITS
  GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,VFS=2,048V, LSB=62.50191F/ 1000000.0
  GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V, VFS=1,024V 15 BITS
  GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,VFS=0,512V 15 BITS
  GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256VVFS=0,256V 15 BITS
  */
  #ifdef EEPROM //Ver la sección GRABA EL DUTY CICLE en el loop
    dutyCicle =dutyCicleBackUp.read();//Lee lo guardado en flash
    if(dutyCicle > 990 || dutyCicle < 10)
    {
      dutyCicle = 990;//Si no está en rango le da el valor mínimo
    }
  #endif
  pwm(PIN_PWM, FREQ_PWM, dutyCicle);
  display_saludo();
  delay(2000); // Presentación inicial del display
  display.clear();
  // Prioridad en las interrupciones. Ver Doc.txt
  NVIC_SetPriority(TCC0_IRQn, 30); // Baja prioridad de interrupción Timer0
  NVIC_SetPriority(EIC_IRQn,31); // Interrupciones de pines. Mínima prioridad 
  NVIC_SetPriority(WDT_IRQn,29); //Interrupción del watchdog
  // https://microchipdeveloper.com/32arm:samd21-nvic-overview
  // Arranca el rotary encoder
  attachInterrupt(CLK_ROTARY_ENCODER, encoder, FALLING);
  // Arranca el Timer0. Empieza a capturar muestras
  TimerTcc0.initialize(TS_20ms); // preferiblemente 20ms
  TimerTcc0.attachInterrupt(timerTS);
  //Habilita el filamento
  digitalWrite(ENABLE_FILAMENTO, HIGH);//Hay una red RC que retarda el enable
  //Programa el watchdog
   #ifdef WATCHDOG
    WatchDog.attachShutdown(WatchDog_reset); // Función ejecutada por el watchdog
   #endif     
  //WDT_HARDCYCLE62m  0x0430    // WDT HARD only : 64 clockcycles @ 1024hz = 62.5ms
  //WDT_HARDCYCLE250m 0x0450    // WDT HARD only : 256 clockcycles @ 1024hz = 250ms
  //WDT_HARDCYCLE1S   0x0470    // WDT HARD only : 1024 clockcycles @ 1024hz = 1 seg
  //WDT_HARDCYCLE2S   0x0480    // WDT HARD only : 2048 clockcycles @ 1024hz = 2 seconds
  //WDT_HARDCYCLE4S   0x0490    // WDT HARD cycle 4 Seconds
  #ifdef WATCHDOG
    //WatchDog.setup(WDT_HARDCYCLE2S);// Activa el watchdog WDT_SOFTCYCLE8S
    WatchDog.setup(WDT_HARDCYCLE1S);
  #endif
}
/**************************************************************
 * loop
 ***************************************************************/
void loop(void)
{
   if (Serial.available()){segaScpi.scpi(&Serial);}
   #ifdef WATCHDOG
    WatchDog.clear(); // Refesca el watchdog-----------------
   #endif
   //ADQUIERE VALORES DEL DAC............................................
   if (leerDAC)
   {
    contadorLecturasADC++;
    digitalWrite(PIN_TEST1, LOW);
    _adc3 = ads.readADC_SingleEnded(3); // Lee el ADC3
    adc3 = adc3 + _adc3;
    _adc0 = ads.readADC_SingleEnded(0); // Lee el ADC0
    adc0 = adc0 + _adc0;
    _adc1 = ads.readADC_SingleEnded(1); // Lee el ADC1
    if ((_adc1 & 0x8000) == 0x8000)
      _adc1 = 0; // Evita valores negativos testeando el bit de signo
    adc1 = adc1 + _adc1;
    _adc2 = ads.readADC_SingleEnded(2); // Lee el ADC2
    adc2 = adc2 + _adc2;
    digitalWrite(PIN_TEST1, HIGH);
    leerDAC = false;
   }
   // ACTUALIZA DUTY CICLE DEL FILAMENTO.....................................
   if(cambiaDutyCicle)//Si es necesario actualiza el setpoint de corriente 
  {
    pwm(PIN_PWM, FREQ_PWM, dutyCicle);
    cambiaDutyCicle=false;
  }
  //ACTUALIZA LAS MEDIDAS EN EL DISPLAY.....................................
  if (contadorLecturasADC == LECTURAS_ADC) // Si ha hecho 20 LECTURAS_ADC promedia y muestra
  {                                        // Inhabilita interrupciones para evitar conflictos
    TimerTcc0.stop();//Detiene el timer de adquisición
    //detachInterrupt(CLK_ROTARY_ENCODER); //Evita interrupciones del rotary encoder
    digitalWrite(PIN_LED_TXL, LOW); // Led para ver la latencia
    digitalWrite(PIN_TEST2, LOW);//2.5ms
    // Calcula valores de medida a partir de las medias de los datos acumulados del ADC
    adc0 = adc0 / LECTURAS_ADC;                 // Corriente de filamento
    adc1 = adc1 / LECTURAS_ADC;                 // Mesh retard
    adc2 = adc2 / LECTURAS_ADC;                 // Corriente Energy (emisión)
    adc3 = adc3 / LECTURAS_ADC;                 // Tensión Energy
    //Calcula los valores reales
    a0 = adc0 * LSB_ADS * GAIN_IFILAMENT; // I_Filamento, Multiplica la palabra digital por el LSB y la ganancia
    a1=  adc1 * PENDIENTE_MR+B_MR; //Recta para calcular el valor real de mesh retard. Ver el excel "Ajuste curvas del DAC.xlsx"
    if(a1<0.0) a1=0.0;//Evita valores negativos
    a2 = adc2 * LSB_ADS * GAIN_IENERGY;   // I_Energy Multiplica la palabra digital por el LSB y la ganancia
    a3=PENDIENTE_VE*adc3+b_VE; //Recta para calcular el valor real de V Energy. Ver el excel "Ajuste curvas del DAC.xlsx"
    // muestra las medidas
    char buffer[16];//Para enviar al display con las medidas
    // filamento
    short porcentajeFilameto=(short)100-(dutyCicle/10);
    snprintf(buffer,sizeof buffer,"I_FIL=%.2fA %d%s",a0,porcentajeFilameto,"% "); //Medida opcional de I fil. donde se ve el %
    //snprintf(buffer,sizeof buffer,"I_FIL=%.2fA  ",a0);
    buffer[15]='\0';//Esto para evitar problemas con los datos enviados al display
    display.drawString(0, 0, buffer);//En la linea 0 I_Filamento a0
    //mesh retard
    snprintf(buffer,sizeof buffer,"M_R=-%.1fV   ",a1);
    buffer[15]='\0';
    display.drawString(0, 2, buffer);//En la linea 2 V_MR a1
    //V Energy
    if(a3<25.0)  snprintf(buffer,sizeof buffer,"V_E>-25.0V  "); //Entre -25V y 0V la fuente no es estable
    else  snprintf(buffer,sizeof buffer,"V_E=-%.1fV   ",a3);  
    buffer[15]='\0';
    display.drawString(0, 4, buffer);
    //I Energy
    a2=a2-a3/16.5;//Le resto la corriente que extraigo para estabilizar la fuente, que no es emisión (R=16,5Koh)
    if (a2<0) a2=0; //Evito medidas por debajo de 0 (offsets)
    snprintf(buffer,sizeof buffer,"I_E=%.2fmA   ",a2); 
    buffer[15]='\0';
    display.drawString(0, 6, buffer);
    // Resetea los acumuladores de medidas  
    adc0 = 0; 
    adc1 = 0;
    adc2 = 0;
    adc3 = 0;
    contadorLecturasADC = 0;// Reset del contador de lecturas
    digitalWrite(PIN_TEST2, HIGH);
    digitalWrite(PIN_LED_TXL, HIGH); // Led para ver la latencia
    TimerTcc0.start();//TimerTcc0.restart() no funciona ¿?
    contadorLoops++;
  }
  //GRABA EL DUTY CICLE SOLO SI HA CAMBIADO
  #ifdef EEPROM
    if(contadorLoops==150) //Si ha pasado un minuto guarda el duty cicle
    {
      if(dutyCicle==dutyCicleAnterior)//Si el duty cicle no ha cambiado en 1 minuto lo guarda en flash si no lo está ya
      {
        if(dutyCicle!=dutyCicleEnFlash)//Si este valor no está ya en flash lo guarda
        {
          dutyCicleBackUp.write(dutyCicle); //Guarda el duty cicle
          dutyCicleEnFlash=dutyCicle; //Recuerda el último valor guardado en flash
          DEPURACION("Guardado en Flash ");
          DEPURACION((int)100-dutyCicle/10);
        }
      }
      else //Si el duty cicle ha cambiado en menos de 1 minuto actualizamos el valor... 
      {
        dutyCicleAnterior=dutyCicle; //... para la siguiente comprobación 
      }
      contadorLoops=0; //Resetea el  contador de loops
    }
  #endif
}
/**************************************************************
 *                FIN
 * ***********************************************************/