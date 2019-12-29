//Iniciamos Librerias
#include <SimpleModbusMaster_DUE.h>
#include <DueTimer.h>
#include <Wire.h>
#include <hd44780.h>                       
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <PID_v1.h>
hd44780_I2Cexp lcd; 

//Viariable del Programa
String inc = "hola"; String incd; char income; 
bool subir ; bool mandar = HIGH; bool curso; bool salir; int i; int e; 
float q = 0.2555;  float brazo = 0.3608; float rpm = 0.0; float fuerza = 0.0; float rpmv, fuerzav, potdesv, potactv;
float potdes = 0; float potact = 0; 
int estfreno;unsigned int freno; unsigned int frenov; unsigned int frenoaut; int estmotor; int paradaemergencia;
const int LCD_COLS = 20;
const int LCD_ROWS = 4;
long previousMillis = 0;
long interval = 200;
bool imprimir;
int var=1;  int valv=0;int var1=1;  int var2=5; int var3=15;  int var4=11;int tiempoespera=30000; int ae=0;
long previousMillis1 = 0; long currentMillis6;
long previousMillis2 = 0; 
long previousMillis3 = 0;
long previousMillis4 = 0; long previousMillis5 = 0;  
long interval2 = 200;
unsigned long currentMilli;

//Variables e inicialización Modbus
#define baud 38400
#define timeout 400
#define polling 150 
#define retry_count 1000000
#define TxEnablePin 2 //pin que controla el transceptor
#define TOTAL_NO_OF_REGISTERS 3
#define retry_connec_ms 2000

//Enumeramos los paquetes del modbus
enum
{
  PACKET1,
  PACKET2,
  TOTAL_NO_OF_PACKETS // Siempre al final
};

Packet packets[TOTAL_NO_OF_PACKETS]; //creamos los paquetes que tendran las configuraciones
unsigned int regs[TOTAL_NO_OF_REGISTERS]; // Registro maestro en array

//Variables e inicialización de PID
double Setpoint, Input, Output, Outputv; int SampleTime = 500;
double aggKp=3, aggKi=0.5, aggKd=0.3;
double consKp=2, consKi=0.4, consKd=0.2;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//Creamos caracteres para dibujar en la pantalla
byte verticalLine[8] = {
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,//--------libr Celda
#include <Wire.h>
#include <hd44780.h>                       
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <SimpleModbusSlave.h>
#include <LiquidCrystal_I2C.h>

//#include <TimerOne.h>
//hd44780_I2Cexp lcd;
//const int LCD_COLS = 16;
//const int LCD_ROWS = 2;

//--------libr Tacometro

#include  <TimerOne.h> 
//#include <eRCaGuy_Timer2_Counter.h>

//-----------Config lcd

LiquidCrystal_I2C lcd(0x27,20,4);

/*********Funciones************/

void promedio_celda ();
void Monitor_LCD();
void envio_datos();
void tacometro();

/**************Seteo de Tiempos*************/ 

//------------------------------Promedio Celda

unsigned long previousMillis = 0;
const long interval = 500 ;

//------------------------------Monitor Datos 

unsigned long previousMillis_1 = 0;
const long interval_1 = 1000 ;

//------------------------------Envio de Datos 

unsigned long previousMillis_2 = 0;
const long interval_2 = 1000 ;


//------------------------------tacometro 

unsigned long previousMillis_3 = 0;
const long interval_3 = 1000 ;

//**************************************//


//-----------CELDA-----------//

#include "HX711.h"
HX711 scale(A1, A0);  // Módulo Celda
double peso_inst = 0; // Variable Peso Instantáneo


// Promedio para Suavizado
const int numReadings = 20;        // tamaño del arreglo
double readings[numReadings];      // lectura de la entrada analógica
int index = 0;                     // el índice de la lectura actual
double average = 0;                // la media
double total = 0;
int thisReading = 0;
unsigned int taux = 0;

//Enumeramos los registros a utilizar (Celda)
              enum   
              {     
                ADC_VAL,
                ADC_VAL_1,     
                PWM_VAL,            
                HOLDING_REGS_SIZE 
              };

unsigned int holdingRegs[HOLDING_REGS_SIZE]; 

//Enumeramos los registros a utilizar (Tacometro)
 /*              
              enum   
             {     
                ADC_VAL_1,     
                PWM_VAL,        
                HOLDING_REGS_SIZE
              };

//unsigned int holdingRegs[HOLDING_REGS_SIZE_1]; 
*/

//-----------TACOMETRO-----------//

volatile boolean output_data = false; //the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long t_start = 0; //units of 0.5us; the input signal high pulse time
volatile unsigned long t_end = 0;
volatile unsigned long t_aux = 0; unsigned long t_aux2 = 0;
float rpm = 0; float rpmv = 0;    long t_while = 0;
volatile unsigned int i=0;

void setup() {
  
  //-------- Config serial
     modbus_configure(&Serial, 38400, SERIAL_8N2, 2, 2, HOLDING_REGS_SIZE, holdingRegs);
     modbus_update_comms(38400, SERIAL_8N2, 2);
  
int status;
  
  //status = lcd.begin(LCD_COLS, LCD_ROWS);
  
  if(status) // non zero status means it was unsuccesful
                          {
    status = -status; 
    hd44780::fatalError(status); // does not return
  }

//-------- Config E/S

  pinMode(7, INPUT); // Boton de TARA
  attachInterrupt(digitalPinToInterrupt(2), ISRe, RISING); // Entrada de interrupciones
 
//-------- Config celda de carga

  scale.set_scale(60419);  // Valor de calibración de la celda de carga
  scale.tare(); // Reset scale a 0 (tara)  

//----Mensaje de Bienvenida----//
  
          lcd.clear();
          lcd.init();
          lcd.backlight();// Indicamos medidas de LC
          lcd.begin(20, 4);
                  
          
          lcd.setCursor(6, 0);
          lcd.print("FRENO PID");  
          lcd.setCursor(4, 1);
          lcd.print("LA 582 LAMyEN"); 
          
          lcd.setCursor(4, 3);
          lcd.print(" Cargando.");
          delay(1000);
          lcd.setCursor(4, 3);
          lcd.print(" Cargando..");
          delay(1000);
          lcd.setCursor(4, 3);
          lcd.print(" Cargando...");
          delay(1000);
          lcd.clear();
         
          lcd.setCursor(6, 0);
          lcd.print("FRENO PID");  
          lcd.setCursor(4, 1);
          lcd.print("LA 582 LAMyEN"); 
          
          lcd.setCursor(4, 3);
          lcd.print(" Cargando.");
          delay(1000);
          lcd.setCursor(4, 3);
          lcd.print(" Cargando..");
          delay(1000);
          lcd.setCursor(4, 3);
          lcd.print(" Cargando...");
          delay(1000);
          lcd.clear();

//-------Monitor Tacometro

          lcd.setCursor(0, 1);
          lcd.print("             "); 
          lcd.setCursor(0, 1);
          lcd.print("Vprom: ");
          lcd.setCursor(13, 1);
          lcd.print(" rpm");

//-------Monitor Torque

          lcd.setCursor(0, 2);
          lcd.print("             "); 
          lcd.setCursor(0, 2);
          lcd.print("Torque:");
          lcd.setCursor(13, 2);
          lcd.print(" kgm/f");

}

void loop() {

//------------------------------Funcion 1 (Promedio Celda)
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    promedio_celda ();
  }


//------------------------------Funcion 2 (Celda - Datos en monitor LCD)
  
  unsigned long currentMillis_1 = millis();

  if (currentMillis_1 - previousMillis_1 >= interval_1) {
    previousMillis_1 = currentMillis_1;
    Monitor_LCD();
  }


//------------------------------Funcion 3 (Envío de Datos)
  
  unsigned long currentMillis_2 = millis();

  if (currentMillis_2 - previousMillis_2 >= interval_2) {
    previousMillis_2 = currentMillis_2;
    envio_datos();
  }

//------------------------------Funcion 4 (tacometro)
  
  unsigned long currentMillis_3 = millis();

  if (currentMillis_3 - previousMillis_3 >= interval_3) {
    previousMillis_3 = currentMillis_3;
    tacometro();
  }


}

// PARA QUE SIRVE ESTE?

void myHandler(){
  }


void promedio_celda(){

    if (digitalRead(7) == HIGH) // Se inicia apretando el boton de TARA
    {
     scale.tare(); // Tara
     average = 0;
     total = 0;
     peso_inst = 0;
            while (thisReading <= numReadings){
            readings[thisReading] = 0;
            thisReading++;
     }
     thisReading = 0; 
    }

     //if(millis()-taux>100){
     //   taux=millis();
    total= total - readings[index];        
    peso_inst = (scale.get_units(3));
    readings[index] = peso_inst;
    total= total + readings[index];      
    index = index + 1;                    

  // si se llego al final de arreglo (al último elemento)
 /* if (index >= numReadings) {  
    lcd.setCursor(7,1);
    lcd.print("       ");           

    index = 0;  
      if (average > -0.0009 && average < 0.0009)
  {
    lcd.setCursor(7,1);
    lcd.print("0.000");
  }
    else
    {
    lcd.setCursor(7,1);
    lcd.print(average,3);
    } 
    lcd.setCursor(14,1);
    lcd.print("kg");  
    }                       
      
    // calcula la media

 
 average = total / numReadings;     
 if (average > -0.0009 && average < 0.0009)
  {
    holdingRegs[ADC_VAL] = 0;
  }  
else
    {
    holdingRegs[ADC_VAL] = average*1000.0;
    } 
  
  // Variable Promedio en Display
   if(millis()-taux>300){modbus_update();}
 */
  }

  void Monitor_LCD(){

    //------------------------------Monitor Fuerza    
              
          //lcd.setCursor(1,0);
          //lcd.print("CELDA DE CARGA");
          lcd.setCursor(0,0); // Se desplazó una línea hacia arriba
          lcd.print("Fuerza: ");
          lcd.setCursor(8,0);
          lcd.setCursor(14,0);
          lcd.print("kg");
    
    if (index >= numReadings) {  
    lcd.setCursor(7,0);
    lcd.print("       ");           

    index = 0;  
      if (average > -0.0009 && average < 0.0009)
            {
              lcd.setCursor(7,0);
              lcd.print("0.000");
            }
                      else
                      {
                      lcd.setCursor(7,0);
                      lcd.print(average,3);// Cantidad de decimales del valor promedio
                      } 
    lcd.setCursor(14,0);
    lcd.print("kg");  
    }   
  }


void tacometro(){

     if(micros()- t_aux2 > 2500000){ rpm=0; lcd.setCursor(6, 1);
     lcd.print("       "); 
     lcd.setCursor(6, 1);lcd.print(rpm); holdingRegs[ADC_VAL_1] = 0;}
     modbus_update();   
     if (output_data==true)
  { t_aux2 = micros();
    rpm = (1200000000.0/(t_end-t_start))/1.000344;
    holdingRegs[ADC_VAL_1] = rpm*10;
       
    if(rpm!=rpmv){
     rpmv=rpm;    
     lcd.setCursor(7, 1);
     lcd.print("       "); 
     lcd.setCursor(7, 1); 
     //lcd.print((rpm/2)/1.00042);
     lcd.print(rpm);
     lcd.setCursor(13, 1);
     lcd.print(" rpm");}
  /*   
     //----------Torque------
     lcd.setCursor(6, 2);
     lcd.print("       "); 
     lcd.setCursor(6, 2); 
     lcd.print(rpm);
     lcd.setCursor(13, 2);
     lcd.print("rpm");}
*/
     output_data = false;
      }  
  } 


void ISRe()   {
              t_aux = micros();
          
              if(output_data==false){
          
              if(i==0){t_start = t_aux; }//0.5uS 
              i++;
              if(i==21){ t_end = t_aux; output_data = true; i=0; }
                        }
              }


void envio_datos() {
             average = total / numReadings;     
             if (average > -0.0009 && average < 0.0009)
              {
                holdingRegs[ADC_VAL] = 0;
              }  
            else
              {
              holdingRegs[ADC_VAL] = average*1000.0;
              } 
  
        // Variable Promedio en Display
         if(millis()-taux>300){modbus_update();}
        
  }
  B00100,
  B00100
};  

byte char2[8] = {
  B00000,
  B00000,
  B00000,
  B11100,
  B00100,
  B00100,
  B00100,
  B00100
};

byte char1[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00111,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};

byte char3[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte char4[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b11100,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

//Iniciamos Bucle de Configuración
void setup()
{
  //Configuración de PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);

  //Configuramos Pines
  pinMode(40, OUTPUT);
  pinMode(42, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(46), descender, RISING);
  attachInterrupt(digitalPinToInterrupt(44), ascender, RISING);
  attachInterrupt(digitalPinToInterrupt(48), ok, RISING);
  attachInterrupt(digitalPinToInterrupt(50), emergencia, FALLING);  
  SerialUSB.begin(250000); 
  analogWriteResolution(15);

  //Configuramos Pantalla
  int status;
  status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) // non zero status means it was unsuccesful
  {
    status = -status; 
    hd44780::fatalError(status); // does not return
  }
  
  // Inicialisamos paquetes de Modbus
  modbus_construct(&packets[PACKET1], 1, READ_HOLDING_REGISTERS, 0, 1, 0);  
  //paquete, ID esclavo, funsión, registro a leer o escribir del esclavo, dejar en "1", direccion donde se guarda)
  modbus_construct(&packets[PACKET2], 2, READ_HOLDING_REGISTERS, 0, 1, 1);
  //modbus_configure(&Serial1, baud, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs, retry_connec_ms);
  modbus_configure(&Serial1, baud, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
  //Imprimimos Pantalla de Presentación
  lcd.setCursor(6,1);
  lcd.print("LAMYEN");
  lcd.setCursor(2,2);
  lcd.print("PID Freno Motor");
   lcd.setCursor(1,2);
  createCustomCharacters();
  printFrame();
  delay(2000);
  Timer2.attachInterrupt(myHandler);
  Timer2.start(300000);
}

 //Función de la interrupción del pin del encoder (giro derecha)
void ascender(){
if(digitalRead(44)){
if(currentMilli - previousMillis1 > interval2 && currentMilli - previousMillis2 > 100) { //Condición que filtra rebote y anula el pin de retroceso
previousMillis2 = currentMilli;
if(!digitalRead(48)){ //Condicion que se cumple cuando se rota el encoder sin presionar ok
if(var>1 && var<5){ //Cuando estamos en modo de seleccion de funcion esto nos deja cambiar la función
switch(var){
    case 2:  var = 3;  break;
    case 3:  var = 4;  break;
    case 4:  var = 2;  break;
    default:    break;
  }
 
  }
  if(var==1){ //Aqui controlamos el valor de brazo y q de calibración
if(var1==1){
        switch(var2){
        case 0: brazo=brazo+1.0;     break;
        case 2: brazo=brazo+0.1;     break; 
        case 3: brazo=brazo+0.01;    break;
        case 4: brazo=brazo+0.001;   break;
        case 5: brazo=brazo+0.0001;  break;}}
if(var1==3){
        switch(var2){
        case 0: q=q+1.0;     break;
        case 2: q=q+0.1;     break;
        case 3: q=q+0.01;    break;
        case 4: q=q+0.001;   break;
        case 5: q=q+0.0001;   break;}}    
}

  if(var==6){ //Aqui controlamos el valor de potencia deseada en modo manual
        switch(var3){
        case 9: potdes=potdes+10000;     break;
        case 10: potdes=potdes+1000;     break;
        case 11: potdes=potdes+100;     break; 
        case 12: potdes=potdes+10;    break;
        case 13: potdes=potdes+1;   break;
        case 15: potdes=potdes+0.1;  break;}}
   if(var==7){
        switch(var4){ //Aqui controlamos el valor de porcentaje de freno en modo manual
        case 7: if(freno+10000<32768){freno=freno+10000;break;}
        case 8: if(freno+1000<32768){freno=freno+1000;break;}
        case 9: if(freno+100<32768){freno=freno+100;break;}
        case 10: if(freno+10<32768){freno=freno+10;break;}
        case 11: if(freno+1<32768){freno=freno+1;break;}}}       
        
        }
else{ //Esta condición solo se cumple cuando se presiona ok y se gira a la vez cumpliendo la función de cambiar el valor que se desea modificar
      // o pasar al modo de selección de función cuando se termina de modificar las variables
if(var==1){
if(var1==3){var=2;}
if(var1==1){var1=3; var2=5;}
}}}}}

void emergencia(){/*analogWrite(13,0); potdes=0; freno=0;*/}

//Función de la interrupción del pin del encoder (giro izquierda)
void descender(){
if(digitalRead(46)){
if(currentMilli - previousMillis2 > interval2 && currentMilli - previousMillis1 > 100) {//Condición que filtra rebote y anula el pin de ascenso
previousMillis1 = currentMilli;
  if(!digitalRead(48)){//Condicion que se cumple cuando se rota el encoder sin presionar ok
if(var==1){ //Aqui controlamos el valor de brazo y q de calibración
if(var1==1){
        switch(var2){
        case 0: brazo=brazo-1;       break;
        case 2: brazo=brazo-0.1;     break; 
        case 3: brazo=brazo-0.01;    break;
        case 4: brazo=brazo-0.001;   break;
        case 5: brazo=brazo-0.0001;  break;}}
if(var1==3){
        switch(var2){
        case 0: q=q-1;       break;
        case 2: q=q-0.1;     break;
        case 3: q=q-0.01;    break;
        case 4: q=q-0.001;   break;
        case 5: q=q-0.0001;   break;}    
}}
if(var==6){ //Aqui controlamos el valor de potencia deseada en modo manual
        switch(var3){
        case 9: if(potdes-10000>-0.01){potdes=potdes-10000;}     break;
        case 10: if(potdes-1000>-0.01){potdes=potdes-1000;}     break;
        case 11: if(potdes-100>-0.01){potdes=potdes-100;}     break;
        case 12: if(potdes-10>-0.01){potdes=potdes-10;}     break;
        case 13: if(potdes-1>-0.01){potdes=potdes-1;}     break;
        case 15: if(potdes-0.1>-0.01){potdes=potdes-0.1;}     break;}}
if(var==7){
        switch(var4){ //Aqui controlamos el valor de porcentaje de freno en modo manual
        case 7: if(freno>9999){
          freno=freno-10000;break;}
        case 8: if(freno>999){freno=freno-1000;break;}
        case 9: if(freno>99){freno=freno-100;break;}
        case 10: if(freno>9){freno=freno-10;break;}
        case 11: if(freno>0){freno=freno-1;break;}}}       
        
if(var>1&&var<5){ //Cuando estamos en modo de seleccion de funcion esto nos deja cambiar la función
switch(var){
    case 2:  var = 4;  break;
    case 3:  var = 2;  break;
    case 4:  var = 3;  break;
    default:    break;
  }}}
   else{ //Esta condición solo se cumple cuando se presiona ok y se gira a la vez

 if(var==1){
if(var1==3&&var2==5){ var1=1; var2=0; } //Cuando estas modificando la segunda variable esta condicion te vuelve a la primera variable
else{
//if(var1==1 && var2==0){var1=3; var2=5; }
if(var1==1 && var2<5){ var2++; if(var2==1){ var2=2;}} //Retrocede un caracter
if(var1==3 && var2<5){ var2++; if(var2==1){ var2=2;}}}//Retrocede un caracter

} 
if(var>1 && var<5){ var=1; } //Cuando estas en la selección de modo esta condición te vuelve a la opción de configuración de variables
if(var>5 && var<8){var=2;}  //Cuando estas ejecutando una función esta condición te devuelve al menu de selección de función
if(var==5){var=2; tiempoespera=0;} 
}}}}

//Función de la interrupción del pin del encoder (ok)
void ok(){
  if(digitalRead(48)){
if(currentMilli - previousMillis3 > 500) { //Condición que filta rebote 
previousMillis3 = currentMilli;
if(var1==3&&var2==0){ var=2; var1=1; var2=5;} //Condición que cuando terminas de modificar las variables te envia al menu de selección de función
if(var==1){
if(var1==1 && var2==0){var1=3; var2=5; } //Condición que cuando terminas de modificar la primera variable te envia a la segunda variable
else{
if(var1==1 && var2>0){ var2--; if(var2==1){ var2=0;}} //Condición para que avance el cursor en la primera variable
if(var1==3 && var2>0){ var2--; if(var2==1){ var2=0;}} //Condición para que avance el cursor en la segunda variable
}}
if(var==6){  //Condición para que avance el cursor en la variable en la función insertar potencia deseada
  switch(var3){ 
   case 15: var3=13; break; 
   case 13: var3=12; break;
   case 12: var3=11; break;
   case 11: var3=10; break;
   case 10: var3=9; break;
   case 9: var3=15; break;      
    }}
if(var==7){  //Condición para que avance el cursor en la variable en la función insertar porcentaje freno
  switch(var4){
   case 11: var4=10; break; 
   case 10: var4=9; break;
   case 9: var4=8; break;
   case 8: var4=7; break;
   case 7: var4=11; break; }}
if(var==2){var=5;} if(var==3){var=6;}if(var==4){var=7;}
}}}

void printFrame() //Función que imprime el marco de la presentación inicial
{ 
  lcd.setCursor(1,0);
  lcd.print("------------------");
  lcd.setCursor(1,3);
  lcd.print("------------------");
  lcd.setCursor(0,1);
  lcd.write(byte(0));
  lcd.setCursor(0,2);
  lcd.write(byte(0));
  lcd.setCursor(19,1);
  lcd.write(byte(0));
  lcd.setCursor(19,2);
  lcd.write(byte(0));
  lcd.setCursor(0,0);
  lcd.write(byte(1));
  lcd.setCursor(19,0);
  lcd.write(byte(2));
  lcd.setCursor(0,3);
  lcd.write(byte(3));
  lcd.setCursor(19,3);
  lcd.write(byte(4));
}

void createCustomCharacters()  //Función que crea los caracteres para el marco de la presentación inicial
{
  lcd.createChar(0, verticalLine);
  lcd.createChar(1, char1);
  lcd.createChar(2, char2);
  lcd.createChar(3, char3);
  lcd.createChar(4, char4);
}

void myHandler(){  //Interrupción por tiempo
  
  if (var==8){SerialUSB.println("D1"); analogWrite(13,0);} //Si entra en modo emergencia envia emergencia al programa y apaga freno
  
  if(var==5 || var==6){ //Condición que se cumple cuando se esta ejecutando la función automatica o potencia mecanica
   /*Configuramos valores indispensables para el PID*/
  Input=potact;
  Setpoint=potdes; 
  double gap = abs(Setpoint-Input); //distancia entre potencia deseada y potencia actual
  if (gap < 500)  //Condición que controla que variable del PID utilizar dependiendo de la distancia entre los valores
  {  myPID.SetTunings(consKp, consKi, consKd);  }
  else
  {  myPID.SetTunings(aggKp, aggKi, aggKd);     } 
  
  if(rpm<250 ){ analogWrite(13,0); Setpoint=0; } //Si la velocidad es menor a 250 rpm apaga el freno
  myPID.Compute();  //Función principal del PID que calcula el valor de salida
  if(digitalRead(50)==LOW || paradaemergencia==1 ){ analogWrite(13,0); var=8; SerialUSB.println("D1");} //Si entra en modo emergencia envia emergencia al programa y apaga freno

  else{if(Output!=Outputv && rpm>250 ){  frenoaut=Output; analogWrite(13,frenoaut); Outputv=Output;}} //Esta condición actualiza la salida del freno  
}

mandar = HIGH; //Se activa el bucle de comunicación con la PC
if(var==1){   //Esta condición actualiza los valores de calibración y controla el cursor
  lcd.setCursor(0,1);
  lcd.print(brazo,4);
  lcd.setCursor(6,1);
  lcd.print(" ");
  lcd.setCursor(5,3);
  lcd.print("  ");
  lcd.setCursor(0,3);
  lcd.print(q,4); 
  if(curso){ lcd.setCursor(var2, var1);lcd.print("_");}
  curso=!curso;}

  if(var==5 ){ //Esta condición actualiza los valores de la función Automatica
    if(rpm!=rpmv){
    rpmv=rpm;
    lcd.setCursor(2,1);lcd.print("       ");
    lcd.setCursor(2,1);lcd.print(rpm,1); }
    if(fuerza!=fuerzav){
    fuerzav=fuerza;
    lcd.setCursor(12,1);lcd.print("       ");
    lcd.setCursor(12,1);lcd.print(fuerza,3);}
    if(potdes!=potdesv){
    potdesv=potdes;
    lcd.setCursor(9,2);lcd.print("           ");
    lcd.setCursor(9,2);lcd.print(potdes,1);}
    if(potact!=potactv){
    potactv=potact;
    lcd.setCursor(9,3);lcd.print("         ");
    lcd.setCursor(9,3);lcd.print(potact,1);
 }}
   if(var==6){  //Esta condición actualiza los valores de la función Pot. Des. y controla el cursor
  if(rpm!=rpmv){
    rpmv=rpm;
    lcd.setCursor(2,1);lcd.print("       ");
    lcd.setCursor(2,1);lcd.print(rpm,1); }
  if(fuerza!=fuerzav){
    fuerzav=fuerza;
    lcd.setCursor(12,1);lcd.print("       ");
    lcd.setCursor(12,1);lcd.print(fuerza,3);}
  if(potact!=potactv){
    potactv=potact;
    lcd.setCursor(9,3);lcd.print("         ");
    lcd.setCursor(9,3);lcd.print(potact,1);}
  
    lcd.setCursor(9,2);lcd.print("           ");
    if(potdes<1 ){ lcd.setCursor(9,2);lcd.print("00000");lcd.setCursor(13,2); lcd.print(potdes,1);}
    else{
    if(potdes<9.9){ lcd.setCursor(9,2);lcd.print("0000");lcd.setCursor(13,2); lcd.print(potdes,1);}
    else{
    if(potdes<99.99){ lcd.setCursor(9,2);lcd.print("000");lcd.setCursor(12,2); lcd.print(potdes,1);}
    else{
    if(potdes<999.99){ lcd.setCursor(9,2);lcd.print("00");lcd.setCursor(11,2); lcd.print(potdes,1);}
    else{
    if(potdes<9999.99){ lcd.setCursor(9,2);lcd.print("0");lcd.setCursor(10,2); lcd.print(potdes,1);}
    else{lcd.setCursor(9,2); lcd.print(potdes,1);}}}}}
    //lcd.setCursor(9,2);lcd.print(potdes,1);
    if(curso){ lcd.setCursor(var3, 2);lcd.print("_");}
    curso=!curso;}


     if(var==7){ //Esta condición actualiza los valores de la función Porc. Freno y controla el cursor
      
     if(freno != frenov){ analogWrite(13,freno); frenov=freno;}
    lcd.setCursor(7,2);lcd.print("       "); 
    if(freno<9.9){ lcd.setCursor(7,2);lcd.print("0000");lcd.setCursor(11,2); lcd.print(freno);}
    else{
    if(freno<99.99){ lcd.setCursor(7,2);lcd.print("000");lcd.setCursor(10,2); lcd.print(freno);}
    else{
    if(freno<999.99){ lcd.setCursor(7,2);lcd.print("00");lcd.setCursor(9,2); lcd.print(freno);}
    else{
    if(freno<9999.99){ lcd.setCursor(7,2);lcd.print("0");lcd.setCursor(8,2); lcd.print(freno);}
    else{lcd.setCursor(7,2); lcd.print(freno);}}}}

    if(curso){ lcd.setCursor(var4, 2);lcd.print("_");}
    curso=!curso;} 
    
 }  //fin interrupción de tiempo

void leer(){ //Bucle que lee y gestiona los datos que provienen de la PC
    if (SerialUSB.available ()> 0)
    {
       inc="";
    }
    while(SerialUSB.available()>0)
    {
    income=((byte) SerialUSB.read());
      inc += income; }} 

void menu(){ //Bucle que controla el menu 
 
switch(var){
    case 1:
  analogWrite(13,0); potdes=0; freno=0;
  valv = var;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Brazo de Palanca");
  lcd.setCursor(0,1);
  lcd.print(brazo,4);
   lcd.setCursor(7,1);
  lcd.print("m");
  lcd.setCursor(0,2);
  lcd.print("Q de calibracion");
  lcd.setCursor(0,3);
  lcd.print(q,4);
  lcd.setCursor(7,3);
  lcd.print("Kg");
      break;
    case 2:
    delay(20);
  analogWrite(13,0); potdes=0; freno=0;
  if(valv==1){
   lcd.clear();
   delay(20);
  lcd.setCursor(0,0);
  lcd.print("Brazo de Palanca");
  lcd.setCursor(0,1);
  lcd.print(brazo,4);
   lcd.setCursor(7,1);
  lcd.print("m");
  lcd.setCursor(0,2);
  lcd.print("Q de calibracion");
  lcd.setCursor(0,3);
  lcd.print(q,4);
  lcd.setCursor(7,3);
  lcd.print("Kg");
  currentMillis6=millis();
  while(millis()-currentMillis6<5000){}}
  valv = var;
  lcd.clear();
  delay(20);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Modo");
  lcd.setCursor(0,1);
  lcd.print("Automatico");
      break;
    case 3:
  lcd.clear();
  analogWrite(13,0); potdes=0; freno=0;
  valv = var;
  delay(20);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Modo");
  lcd.setCursor(0,1);
  lcd.print("Manual Cargar ");
 lcd.setCursor(0,2);
 lcd.print("Potencia");

      break;
    case 4:
  lcd.clear();
  valv = var;
  delay(20);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Modo");
  lcd.setCursor(0,1);
  lcd.print("Manual %Freno");
      break;   
    case 5:
  lcd.clear();
  delay(20);
  valv = var;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Freno Automatico");
  lcd.setCursor(0,1);
  lcd.print("V:        F:");
  lcd.setCursor(0,2);
  lcd.print("Pot.Des.:");
  lcd.setCursor(0,3);
  lcd.print("Pot.Atc.:");
  lcd.setCursor(2,1);lcd.print(rpm,1); 
  lcd.setCursor(12,1);lcd.print(fuerza,3);
  lcd.setCursor(9,2);lcd.print(potdes,1);
  lcd.setCursor(9,3);lcd.print(potact,1);
      break;
       case 6:
  lcd.clear();
  delay(20);
  valv = var;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Freno Manual Pot.");
  lcd.setCursor(0,1);
  lcd.print("V:        F:");
  lcd.setCursor(0,2);
  lcd.print("Pot.Des.:");
  lcd.setCursor(0,3);
  lcd.print("Pot.Atc.:");
  lcd.setCursor(2,1);lcd.print(rpm,1); 
  lcd.setCursor(12,1);lcd.print(fuerza,3);
  lcd.setCursor(9,2);lcd.print(potdes,1);
  lcd.setCursor(9,3);lcd.print(potact,1);
      break;
       case 7:
  lcd.clear();
  delay(20);
  valv = var;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Freno Manual %");
  lcd.setCursor(0,2);
  lcd.print("%Freno:");
      break;
      case 8:
  lcd.clear();
  delay(20);
  valv = var;
  lcd.clear();
  lcd.setCursor(4,1);
  lcd.print("EMERGENCIA!!");
  lcd.setCursor(2,2);
  lcd.print("Reiniciar Freno");
  printFrame();
      break;
  }}

void loop()
{
  /*Condición que controla el estado de emegencia*/
  if(digitalRead(50)==LOW || paradaemergencia==1 || var==8){analogWrite(13,0);digitalWrite(42, HIGH);digitalWrite(40,LOW);} 
  else {digitalWrite(40,HIGH);digitalWrite(42,LOW);}
  
  if(valv != var){menu();} //Cuando se cambia el estado de la pantalla, se llama esta función que controla los perfiles principales del display
  
  currentMilli = millis();
  /*Otra condición complementaria que controla el estado de emegencia*/
  if(!digitalRead(50)){potdes=0;freno=0; var=8;analogWrite(13,0);}
  
  if(var==5 || var==6){
  modbus_update();  //funcion que actualiza los paquetes del modbus
  /*Aqui se acondicionan los valores recibidos de los esclavos*/
  rpm = (regs[0]/10.0); fuerza = regs[1]/1000.0; 
  if(fuerza>50){fuerza=0;}
  
  potact=(6.28/60.0)*9.81*(q+fuerza)*rpm*brazo; //Se calcula la potencia actual
                      }
/*Función que envia los valores a la PC*/
if(var==5){ 
if(mandar == HIGH){

SerialUSB.print("A");
SerialUSB.println(rpm); //Rpm
delay(10);
SerialUSB.print("B");
SerialUSB.println(fuerza,3); //Carga
delay(10);
SerialUSB.print("C"); //Pot Mec
SerialUSB.println(potact);
delay(10);
if(!digitalRead(50)){SerialUSB.println("D1");}
else {SerialUSB.println("D0");} 
delay(10);
SerialUSB.println("E0");
delay(10);
SerialUSB.println("F1"); 

inc = "hola"; 
mandar = LOW;
}
previousMillis4=millis();previousMillis5=millis(); salir=LOW; tiempoespera=30000;

/*Este bucle queda funcionando cada ves que se llama hasta que termine de recivir todos los datos
  Tiene un tiempo limite de 30 seg por si se pierde la comunicación */
while(inc.substring(0,3) != "fin" || salir==HIGH ){
inc=" ";
leer();
currentMilli=millis();
if(currentMilli-previousMillis4>tiempoespera){var=2;inc="fin";}
        if (inc.substring(0,1) == "H") {
    incd=inc.substring(1, 6);    
  estfreno = incd.toInt(); 
  }
        if (inc.substring(0,1) == "I") {
    incd=inc.substring(1, 6);    
  paradaemergencia = incd.toInt(); }
          if (inc.substring(0,1) == "J") {
    incd=inc.substring(1, 6);    
  estmotor = incd.toInt(); }
    if (inc.substring(0,1) == "G") {       
  if(digitalRead(50)==LOW || paradaemergencia == 1 || currentMilli-previousMillis5>700){ 
  potdes = 0; inc=" ";inc="fin";digitalWrite(40,HIGH);digitalWrite(42,HIGH);}else{incd=inc.substring(1, 6);potdes = incd.toInt();}}
/*Si la comunicación tarda mas de 700 mS apaga el freno */
if(digitalRead(50)==LOW || paradaemergencia == 1 || currentMilli-previousMillis5>700){potdes = 0;  analogWrite(13,0);inc=" ";inc="fin";digitalWrite(40,HIGH);digitalWrite(42,HIGH);}}    
 }}

 




