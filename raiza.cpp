/* 
1. alat sudah siap

tampilkan berat beras

2. takaran air : >under >medium >over = click
 proses:
 1. isi air jika sesuai takaran cuci valve in nyala
 2. tampilkan berat 
 3. motor pengaduk selama beberapa detik
 4. pt dioda read sens tampilkan data
 5. jika air masih kotor 
 6. valve out nyala
 7. tampilkan berat jika berat < takaran awal maka 
 8. [1]
 9. [4] jika air sudah jernih maka: [6][7]
 10. isi air jika sesuai takaran setingan valve in nyala
 hasil
*/
#include "HX711.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "stdio.h"
#include "math.h"

#define LOADCELL_DOUT_PIN A0
#define LOADCELL_SCK_PIN A1
#define PT_dioda A2
#define pin_pwm_motor 3
#define pin_motor_a 2
#define pin_motor_b 4
#define SW 10
#define Valve_in 13
#define Valve_out 12
//
int counter = 0;
int pushed = 0; 
int control = 0;
int last_counter = 0;
int menu = 0;
int awal = 0;
int next = 0; 
bool clk_State;
bool Last_State; 
bool dt_State; 
float cal_ptsens = 0;
float PD_sens = 0;
float berat = 0;
float berat_awal = 0;
unsigned long waktu_isi_awal = 0;
//
LiquidCrystal_I2C lcd(0x27, 16, 2);
uint8_t arrow[8] = {0x00, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00};

HX711 scale;

void initLoadcel()
{
  Serial.println("______________________");
  Serial.println("Calibrasi Load Sensor");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(440.0);
  scale.tare(50);
  Serial.println("Calibrasi Berhasil");
  Serial.println("______________________");
  lcd.createChar(0, arrow);
}
void initPtdiod()
{
  while (millis() < 5000){
    unsigned int adcIn=analogRead(PT_dioda);
    float pt_calsens = ((float)adcIn/1024)*1.1*100;
    cal_ptsens += pt_calsens;
    Serial.print("V_Photodioda: "); 
	Serial.println(cal_ptsens);
    return cal_ptsens;
  }
  cal_ptsens /= 10;
}
void loadcel();

void setup() 
{
  Serial.begin(9600);
  lcd.init(); 
  lcd.backlight();
  lcd.createChar(1, arrow); 
  lcd.home();
  pinMode(Valve_in, OUTPUT);
  pinMode(Valve_out, OUTPUT);
  pinMode(pin_motor_a, OUTPUT);
  pinMode(pin_motor_b, OUTPUT);
  pinMode(pin_pwm_motor, OUTPUT);
  pinMode(SW, INPUT);
  
  PCICR |= (1 << PCIE0);                                               
  PCMSK0 |= (1 << PCINT0); 
  PCMSK0 |= (1 << PCINT1);
  DDRB &= B11111100;
  Last_State =   (PINB & B00000001);

  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.write(0); 
  lcd.print(" ELEKTRONIKA A ");
  lcd.write(0); 
  lcd.setCursor(0,1);  
  lcd.print(" CLEANING RICE ");
  delay(1000);
  
  Serial.println("_____________________"); 
  Serial.println();
  Serial.println("Callibrasi Sensor dan inisialisasi data");
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.write(1); 
  lcd.print("  Calibrasi ");
  lcd.setCursor(0,1);
  lcd.print(" Loadcell Ptdio");
  initLoadcel(); 
  //initPtdiod();
  
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.write(1); 
  lcd.print("  Calibrasi ");
  lcd.setCursor(0,1);
  lcd.print("  Berhasil");
  Serial.println("Callibrasi Sensor berhasil dan siap");
  Serial.println("_____________________"); 
  Serial.println();
  delay(2000); 
  
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print(" CLEANING RICE ");
  lcd.setCursor(0,1); 
  lcd.print("SIAP <klik OK> PB");
  delay(1000); 
}

void loop() 
{
  // program runing in this scope
if (control == 0)
{
    if (pushed ==1)
	{
    Serial.println("Next >>>0");
    delay(500);
    control = 1; 
	pushed = 0;
   }
}



if (control == 1)
{
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print(" CLEANING RICE ");
  next = 1; 
  loadcel();
  if (pushed ==1)
  {
    Serial.println("Next >>>");
    delay(500); 
	lcd.clear();
    control = 2; 
	pushed = 0;
    berat_awal = berat;
   }
 }
 
 
 
  if((last_counter > counter) || (last_counter < counter)  || pushed){
    if (control == 2)
	{
      Serial.print("Position:"); 
	  Serial.println(counter);
      if(0 <= counter && counter < 2)
	  {
        lcd.clear(); 
		lcd.setCursor(0,0);
        lcd.write(1);
		lcd.print("Under Cook");
        lcd.setCursor(0,1);
		lcd.print(" Medium Cook");
        Serial.print("Menu 1"); 
		menu = 1;
        if (pushed ==1)
		{
          Serial.println("Under Cook");
          delay(500);
          control = 3; 
		  pushed = 0;
        }
      }
      if(2 <= counter && counter < 4)
	  {
        lcd.clear(); 
		lcd.setCursor(0,0);
        lcd.print(" Under Cook"); 
		lcd.setCursor(0,1);
        lcd.write(1); 
		lcd.print("Medium Cook");
        Serial.print("Menu 2"); 
		menu = 2;
        if (pushed ==1)
		{
          Serial.println("Medium Cook");
          delay(500);
          control = 3; 
		  pushed = 0;
        }
      }
      if(4 <= counter && counter < 6)
	  {
        lcd.clear(); 
		lcd.setCursor(0,0);
        lcd.write(1); 
		lcd.print("Over Cook");
        lcd.setCursor(0,1); 
		lcd.print("<Back Menu");
        Serial.print("Menu 3"); 
		menu = 3;
        if (pushed ==1)
		{
          Serial.println("Over Cook");
          delay(500);
          control = 3; 
		  pushed = 0;
        }
      }
      if(6<= counter && counter < 8)
	  {
        lcd.clear(); 
		lcd.setCursor(0,0);
        lcd.print(" Over Cook"); 
		lcd.setCursor(0,1); 
        lcd.write(1); 
		lcd.print("<Back Menu");
        Serial.print("Menu 4"); 
		menu = 4;
        if (pushed ==1)
		{
          Serial.println("<<Back");
          delay(500); 
		  menu = 0;
          control = 1; 
		  pushed = 0;
        }
      }
    }
  }
  
  
  
  if(digitalRead(10) >0)
  {
    pushed = 1;
  }
  if (control == 3)
  {
    digitalWrite(Valve_in, HIGH);
    lcd.clear(); 
	lcd.setCursor(0,0); 
    lcd.print(" PENGISIAN AIR "); 
	Serial.println(" PENGISIAN AIR ");
    loadcel(); 
    Serial.print(berat, 1); 
	Serial.println(" g");
    if (berat > 3000)
	{
      digitalWrite(Valve_in, LOW);
      lcd.clear();
      lcd.setCursor(0,0); 
	  lcd.print(" PENGISIAN AIR ");
      lcd.setCursor(0,1); 
	  lcd.print("   SELESAI ");
      control = 4;
    }
  }
  
  
  if (control == 4)
  {
    PD_sens = getPTdioda(PT_dioda);
    bool arah_putaran = 1; 
    uint8_t kecepatan_motor = 225;
    motor_dc(arah_putaran, kecepatan_motor); 
    Serial.print("Jernih:"); 
	Serial.print(PD_sens, 1); 
    Serial.println(" air"); 
	lcd.clear();
    lcd.setCursor(0,0); 
	lcd.print(" MOTOR MENYALA ");
    lcd.setCursor(0,1); 
	lcd.print(" Jernih = ");
    lcd.setCursor(9,1); 
	lcd.print(PD_sens, 1);
    unsigned long waktu_isi_sekarang = millis();
    if (waktu_isi_sekarang - waktu_isi_awal >= 4000)
	{ //berputar selama 4 detik
      arah_putaran = 0; 
	  kecepatan_motor = 0;
      motor_dc(arah_putaran, kecepatan_motor);
      control = 5; 
      lcd.clear();
      lcd.setCursor(0,0); 
	  lcd.print(" MOTOR Mati ");
      lcd.setCursor(0,1); 
	  lcd.print(" Jernih = ");
      lcd.setCursor(9,1); 
	  lcd.print(PD_sens, 1);
    }
  }
  
  
  
  if (control == 5)
  {
    digitalWrite(Valve_out, HIGH);
    lcd.clear(); 
	lcd.setCursor(0,0); 
    lcd.print(" PENGOSONGAN AIR "); 
	Serial.println(" PENGOSONGAN AIR ");
    loadcel(); 
    Serial.print(berat, 1); 
	Serial.println(" g");
    float berat_out = berat_awal + 500;
    if (berat < berat_out)
	{
      digitalWrite(Valve_out, LOW);
      lcd.clear();
      lcd.setCursor(0,0); 
	  lcd.print(" PENGISIAN AIR ");
      lcd.setCursor(0,1); 
	  lcd.print("   SELESAI ");
      control = 6;
    }
  }
  
  
  
  if (control == 6)
  {
    if (PD_sens >= 0)
	{ //kondisi jika tidak jernih
     control = 3; 
	 lcd.clear(); Serial.println(" AIR BELUM JERNIH");
      lcd.setCursor(0,0); 
	  lcd.print("AIR BELUM JERNIH");
      lcd.setCursor(0,1); 
	  lcd.print(" Jernih = ");
      lcd.setCursor(9,1); 
	  lcd.print(PD_sens, 1);
    } 
	else 
	{
      control = 7;
      lcd.clear(); 
	  Serial.println(" AIR SUDAH JERNIH");
      lcd.setCursor(0,0); 
	  lcd.print("AIR DUDAH JERNIH");
      lcd.setCursor(0,1); 
	  lcd.print(" Jernih = ");
      lcd.setCursor(9,1); 
	  lcd.print(PD_sens, 1);
    }
  }
  
  
  if (control == 7)
  {
    if (menu == 1)
	{
      digitalWrite(Valve_in, HIGH);
      lcd.clear(); 
	  lcd.setCursor(0,0);
      lcd.print("MODE UNDER COOK"); 
	  Serial.println("MODE UNDER COOK");
      loadcel(); 
      Serial.print(berat, 1); 
	  Serial.println(" g");
      if (berat >= 4000)
	  { // PERHITUNGAN UNDER COOK DENGAN PENGAMBILAN DATA
        digitalWrite(Valve_in, LOW);
        lcd.clear();
        lcd.setCursor(0,0); 
		lcd.print(" PENGISIAN AIR ");
        lcd.setCursor(0,1); 
		lcd.print("   SELESAI ");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0,0); 
		lcd.print("RICE UNDER COOK");
        lcd.setCursor(0,1); 
		lcd.print("   SELESAI ");
        control = 0;
      }
    }
	
    if (menu == 2)
	{
      digitalWrite(Valve_in, HIGH);
      lcd.clear(); 
	  lcd.setCursor(0,0);
      lcd.print("MODE MEDIUM COOK"); 
	  Serial.println("MODE MEDIUM COOK");
      loadcel(); 
      Serial.print(berat, 1); 
	  Serial.println(" g");
      if (berat >= 3000)
	  {
        digitalWrite(Valve_in, LOW);
        lcd.clear();
        lcd.setCursor(0,0); 
		lcd.print(" PENGISIAN AIR ");
        lcd.setCursor(0,1); 
		lcd.print("   SELESAI ");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0,0); 
		lcd.print("RICE MEDIUM COOK");
        lcd.setCursor(0,1); 
		lcd.print("   SELESAI ");
        control = 0;
      }
    }
    if (menu == 3)
	{
      digitalWrite(Valve_in, HIGH);
      lcd.clear(); 
	  lcd.setCursor(0,0);
      lcd.print("MODE OVER COOK"); 
	  Serial.println("MODE OVER COOK");
      loadcel(); 
      Serial.print(berat, 1); 
	  Serial.println(" g");
      if (berat >= 2000)
	  {
        digitalWrite(Valve_in, LOW);
        lcd.clear();
        lcd.setCursor(0,0); 
		lcd.print(" PENGISIAN AIR ");
        lcd.setCursor(0,1); 
		lcd.print("   SELESAI ");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0,0); 
		lcd.print("RICE OVER COOK");
        lcd.setCursor(0,1); 
		lcd.print("   SELESAI ");
        control = 0;
      }
    }
  }
  last_counter = counter;
  
  
  if(counter > 8)
  { 
  counter=8;
  }
  if(counter < 0) 
  { 
  counter=0; 
  }
}
void loadcel()
{
  Serial.print("Berat:");
  scale.power_up();
  berat = scale.get_units(25);
  if (berat <= 0.1)
  {
    berat = 0.0; 
	Serial.print(berat, 1); 
	Serial.println(" g");
    lcd.setCursor(0,1); 
	lcd.print(" Berat= ");
    lcd.setCursor(9,1); 
	lcd.print(berat, 1);
    lcd.print(" g");
  }
  if (berat < 1000)
  {
    Serial.print(berat, 1); 
	Serial.println(" g");
    lcd.setCursor(0,1); 
	lcd.print(" Berat= ");
    lcd.setCursor(9,1); 
	lcd.print(berat, 1);
    lcd.print(" g");
  }
  if (berat >= 1000)
  {
    float berat_kg = berat / 1000;
    Serial.print(berat_kg, 1); 
	Serial.println(" Kg");
    lcd.setCursor(0,1); 
	lcd.print(" Berat= ");
    lcd.setCursor(9,1); 
	lcd.print(berat, 1);
    lcd.print(" Kg");
  }
  scale.power_down(); 
  
  //delay(1000); scale.power_up();
}

float getPTdioda(unsigned int dioda)
{
  unsigned int adcIn=analogRead(dioda);
  float data_Pt_dioda=((float)adcIn/1024)*1.1*100;//>> ubah rumus baca sensor  
  return data_Pt_dioda;
}

void motor_dc(bool arah, uint8_t pwm)
{
  analogWrite(pin_pwm_motor, pwm);
  
  switch(arah)
  {
    case 0:
      digitalWrite(pin_motor_a, HIGH);
      digitalWrite(pin_motor_b, LOW);
      break;
    case 1:
      digitalWrite(pin_motor_a, LOW);
      digitalWrite(pin_motor_b, HIGH);
      break;
  }
}

//-----------------------------Menu control sistem
ISR(PCINT0_vect)
{
  clk_State =   (PINB & B00000001); //pin8 interupt
  dt_State  =   (PINB & B00000010); //pin7 interupt
  
  if (clk_State != Last_State)
  {     
     if (dt_State != clk_State) 
	 { 
       counter ++;
     }
     else {
       counter --;
     } 
   } 
   Last_State = clk_State;
}
