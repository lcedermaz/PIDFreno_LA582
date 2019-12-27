// codigo que junta las rpm y la medicion de la celda de carga para obtener el torque del equipo
// de esta manera eliminamos 1 display, un rs485, un arduino uno. Reduciendo conexiones y posibilidades de ruido

#include <Wire.h>
#include <hd44780.h>                       
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <SimpleModbusSlave.h>
#include <LiquidCrystal_I2C.h>

//#include <TimerOne.h>
//hd44780_I2Cexp lcd;
//const int LCD_COLS = 16;
//const int LCD_ROWS = 2;


//-----------Config lcd

LiquidCrystal_I2C lcd(0x27,20,4);

/*********Funciones************/

void promedio_celda ();
void Monitor_LCD();
void envio_datos();


/**************Seteo de Tiempos*************/ 

//------------------------------Promedio Celda

unsigned long previousMillis = 0;
const long interval = 500 ;

//------------------------------Monitor Datos 

unsigned long previousMillis_1 = 0;
const long interval_1 = 1000 ;

//------------------------------Envio de Datos 

unsigned long previousMillis_2 = 0;
const long interval_2 = 50 ;

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

enum   //Enumeramos los registros a utilizar
{     
  ADC_VAL,            
  HOLDING_REGS_SIZE 
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; 


//-----------TACOMETRO-----------//


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
