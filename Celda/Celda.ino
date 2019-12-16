#include <Wire.h>
//#include <TimerOne.h>
#include <hd44780.h>                       
#include <hd44780ioClass/hd44780_I2Cexp.h>
hd44780_I2Cexp lcd;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
#include <SimpleModbusSlave.h>

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  //Direccion de LCD

#include "HX711.h"
HX711 scale(A1, A0);  // Módulo Celda
double peso_inst = 0; // Variable Peso Instantáneo

// Promedio para Suavizado
const int numReadings = 20;     // tamaño del arreglo
double readings[numReadings];      // lectura de la entrada analógica
int index = 0;                  // el índice de la lectura actual
double average = 0;              // la media
double total = 0;
int thisReading = 0;
unsigned int taux = 0;

enum   //Enumeramos los registros a utilizar
{     
  ADC_VAL,            
  HOLDING_REGS_SIZE 
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; 


void setup()
{
  modbus_configure(&Serial, 38400, SERIAL_8N2, 2, 2, HOLDING_REGS_SIZE, holdingRegs);
  modbus_update_comms(38400, SERIAL_8N2, 2);
  
int status;
  status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) // non zero status means it was unsuccesful
  {
    status = -status; 
    hd44780::fatalError(status); // does not return
  }

// Pulsador para Tara
  pinMode(7, INPUT);

// Inicia el lcd
  //lcd.begin(16,2);   
 // lcd.home();
  lcd.setCursor(5,0);
  lcd.print("LAMYEN");
  lcd.setCursor(2,1);
  lcd.print("Iniciando...");
  delay(3000);

  scale.set_scale(60419);  // Valor de calibración de la celda de carga
  scale.tare(); // Reset scale a 0 (tara)
  lcd.clear();

  lcd.setCursor(3,0);
  lcd.print("- Listo -");
  delay(1000);
  lcd.clear();
  
lcd.setCursor(1,0);
  lcd.print("CELDA DE CARGA");
  lcd.setCursor(0,1);
  lcd.print("Fuerza:");
  lcd.setCursor(8,1);
    lcd.setCursor(14,1);
  lcd.print("kg");
 // Timer1.attachInterrupt(myHandler);
//  Timer1.initialize(300000);

}


void myHandler(){

}



void loop()
{


// Pulsador para Tara
    if (digitalRead(7) == HIGH) 
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
  // lectura del sensor
  peso_inst = (scale.get_units(3));
  readings[index] = peso_inst;
  // suma la lectura actual y el total
  total= total + readings[index];      
  // avanza al siguiente elemento del arreglo
  index = index + 1;                    

  // si se llego al final de arreglo (al último elemento)
  if (index >= numReadings) {  
    lcd.setCursor(7,1);
  lcd.print("       ");           
    // se vuelve al primer elemento (índice 0)
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

    }

