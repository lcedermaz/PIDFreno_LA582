//#include <eRCaGuy_Timer2_Counter.h>
#include <SimpleModbusSlave.h>
#include <LiquidCrystal.h>
#include  <TimerOne.h> 
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

enum   //Enumeramos los registros a utilizar
{     
  ADC_VAL,     
  PWM_VAL,        
  HOLDING_REGS_SIZE 
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; 

//const byte INPUT_PIN = 12; 
                           
volatile boolean output_data = false; //the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long t_start = 0; //units of 0.5us; the input signal high pulse time
volatile unsigned long t_end = 0;
volatile unsigned long t_aux = 0; unsigned long t_aux2 = 0;
float rpm = 0; float rpmv = 0;    long t_while = 0;
volatile unsigned int i=0;



void setup() 
{
 modbus_configure(&Serial, 38400, SERIAL_8N2, 1, 3, HOLDING_REGS_SIZE, holdingRegs);
  modbus_update_comms(38400, SERIAL_8N2, 1);
  //Serial.begin(115200);
  //pinMode(INPUT_PIN,INPUT_PULLUP); //use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  attachInterrupt(digitalPinToInterrupt(2), ISRe, RISING);
 
 lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("   TACOMETRO");
  lcd.setCursor(0, 1);
  delay(750);
  lcd.print("     LAMYEN");
  delay(750);
    lcd.setCursor(0, 1);
      lcd.print("               ");
      lcd.setCursor(0, 1);
  lcd.print("Cargando");
  delay(500);   
      lcd.setCursor(0, 1);
      lcd.print("               ");
  lcd.setCursor(0, 1);
  lcd.print("Cargando.");
  delay(500);
      lcd.setCursor(0, 1);
      lcd.print("               ");
  lcd.setCursor(0, 1);
  lcd.print("Cargando..");
  delay(500);
      lcd.setCursor(0, 1);
      lcd.print("               ");
    lcd.setCursor(0, 1);
  lcd.print("Cargando...");
  delay(500);   
      lcd.setCursor(0, 1);
      lcd.print("               ");
      lcd.setCursor(0, 1);
  lcd.print("Cargando");
  delay(500); 
      lcd.setCursor(0, 1);
      lcd.print("               ");  
  lcd.setCursor(0, 1);
  lcd.print("Cargando.");
  delay(500);
      lcd.setCursor(0, 1);
      lcd.print("               ");
  lcd.setCursor(0, 1);
  lcd.print("Cargando..");
  delay(500);
      lcd.setCursor(0, 1);
      lcd.print("               ");
    lcd.setCursor(0, 1);
  lcd.print("Cargando...");
  delay(500); 

      lcd.setCursor(0, 1);
  lcd.print("             "); 
    lcd.setCursor(0, 1);
  lcd.print("Vprom:");
     lcd.setCursor(13, 1);
   lcd.print("rpm");
  
}


void loop() 
{

if(micros()- t_aux2 > 2500000){ rpm=0; lcd.setCursor(6, 1);
     lcd.print("       "); 
     lcd.setCursor(6, 1);lcd.print(rpm); holdingRegs[ADC_VAL] = 0;}
modbus_update();   
  if (output_data==true)
  { t_aux2 = micros();
    rpm = (1200000000.0/(t_end-t_start))/1.000344;
    holdingRegs[ADC_VAL] = rpm*10;
       
    if(rpm!=rpmv){
     rpmv=rpm;    
     lcd.setCursor(6, 1);
     lcd.print("       "); 
     lcd.setCursor(6, 1); 
     //lcd.print((rpm/2)/1.00042);
     lcd.print(rpm);
     lcd.setCursor(13, 1);
     lcd.print("rpm");}

    output_data = false;
  }} 

void ISRe()
{
    t_aux = micros();

    if(output_data==false){

    if(i==0){t_start = t_aux; }//0.5uS 
    i++;
    if(i==21){ t_end = t_aux; output_data = true; i=0; }
    }}


