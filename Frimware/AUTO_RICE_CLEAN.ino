/* https://how2electronics.com/diy-turbidity-meter-using-turbidity-sensor-arduino/ 
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
#include <FahmiKalmanFilter.h>
#include "stdio.h"
#include "math.h"

//PID
#define KP 2.0
#define KI 0.5
#define KD 0.8
//Inisialisasi
#define LOADCELL_DOUT_PIN A0
#define LOADCELL_SCK_PIN A1
#define Turbidity_pin A2
#define pin_pwm_motor 6
#define pin_motor_a 7
#define pin_motor_b 11
#define pin_pwm_pompa 5
#define pin_pompa_a 4
#define pin_pompa_b 3
#define SW 10
#define Valve_in 2
#define Valve_out 13
//
int counter = 0, pushed = 0, control = 0;
int control2 = 0;
int last_counter = 0, menu = 0, awal = 0;
int next = 0; 
bool clk_State;
bool Last_State; 
bool dt_State; 
float cal_ptsens = 0;
float PD_sens = 0;
float berat = 0, berat_awal = 0;
unsigned long waktu_isi_awal = 0;
float volt, ntu;
//
//PID
float eror_kec, previous_eror;
float PID_P, PID_I, PID_D, PID_TOTAL;
float time_pid;
int period_pid = 50;
float serpoint_berat = 65;
LiquidCrystal_I2C lcd(0x27, 16, 2);
uint8_t arrow[8] = {0x00, 0x04 ,0x06, 0x1f, 0x06, 0x04, 0x00};

HX711 scale;

void loadcel();

void setup() {
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
  lcd.print("Rayza Rizky F P");
  lcd.write(0); 
  lcd.setCursor(0,1);  
  lcd.print(" 1103181027 ");
//  delay(1000);
//  Serial.println("_____________________"); Serial.println();
//  Serial.println("Callibrasi Sensor dan inisialisasi data");
//  lcd.clear();  lcd.setCursor(0,0);
//  lcd.write(1); lcd.print("  Calibrasi ");
//  lcd.setCursor(0,1);lcd.print(" Loadcell Ptdio");
  Serial.println("______________________");
  Serial.println("Calibrasi Load Sensor");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(440.0);
  scale.tare(50);
  lcd.createChar(0, arrow);
//  lcd.clear();  lcd.setCursor(0,0);
//  lcd.write(1); lcd.print("  Calibrasi ");
//  lcd.setCursor(0,1);lcd.print("  Berhasil");
//  Serial.println("Calibrasi Sensor berhasil dan siap");
//  Serial.println("_____________________"); Serial.println();
  delay(1000); 
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Pencuci Beras");
  lcd.setCursor(0,1); 
  lcd.print("Siap? <klik OK> ");
  delay(1000); 
}

void loop() {
  // program runing in this scope
if(digitalRead(SW)<1){
    pushed = 1;
  }
if (control == 0){
    if (pushed ==1){
    Serial.println("Berikutnya >");
    delay(500); 
    lcd.clear();
    control = 1; 
    pushed = 0;
   }
}
if (control == 1){
  lcd.setCursor(0,0); 
  lcd.print("Pencuci Beras");
  next = 1; 
  loadcel(); 
  //control = 4; waktu_isi_awal = millis();
  if (pushed == 1){
    Serial.println("Berikutnya >");
    delay(500); 
    lcd.clear();
    control = 2;
    pushed = 0;
    berat_awal = berat;
   }
 }
 
 if (control == 2){
   if((last_counter > counter) || (last_counter < counter)){
    Serial.print("Takaran: "); 
    Serial.println(counter);
    if(0 <= counter && counter < 2){
      lcd.clear(); 
      lcd.setCursor(0,0);
      lcd.write(1);
      lcd.print("Kurang matang");
      lcd.setCursor(0,1);
      lcd.print("Matang");
      
      if (pushed == 1){
        menu = 1;
        Serial.println("Kurang matang");
        Serial.print("Menu 1");
        delay(100); 
        control = 3; 
        pushed = 0;
        lcd.clear();
      }
    }
    
    if(2 <= counter && counter < 4){
      lcd.clear(); 
      lcd.setCursor(0,0);
      lcd.print("Kurang matang"); 
      lcd.setCursor(0,1);
      lcd.write(1); 
      lcd.print("Matang");
      
      if (pushed ==1){
        Serial.println("Matang");
        Serial.print("Menu 2");
        delay(100); 
        menu = 2;
        control = 3; 
        pushed = 0;
        lcd.clear();
      }
    }
    
    if(4 <= counter && counter < 6){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write(1);
      lcd.print("Terlalu Matang");
      lcd.setCursor(0,1);
      lcd.print("< Kembali");
      
      if (pushed ==1){
        Serial.println("Terlalu Matang");
        Serial.print("Menu 3");
        delay(100); 
        menu = 3;
        control = 3;
        pushed = 0;
        lcd.clear();
      }
    }
    
    if(6<= counter && counter < 8){
      lcd.clear(); 
      lcd.setCursor(0,0);
      lcd.print("Terlalu Matang"); 
      lcd.setCursor(0,1); 
      lcd.write(1); 
      lcd.print("< Kembali");
      
      if (pushed ==1){
        Serial.println("< Kembali"); 
        Serial.print("Menu 4"); 
        delay(100); 
        menu = 0;
        control = 1; 
        pushed = 0;
      }
    }
  }
 }
 
  if (control == 3){
    digitalWrite(Valve_in, HIGH);
    loadcel();
    lcd.setCursor(0,0); 
    lcd.print(" Pengisian Air "); 
    Serial.println(" Pengisian Air "); 
    Serial.print(berat, 1); 
    Serial.println(" g");
    
    if (berat >= 1700){
      digitalWrite(Valve_in, LOW);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" Pengisian Air ");
      lcd.setCursor(0,1); 
      lcd.print("   Selesai ");
      delay(1000);
      waktu_isi_awal = millis();
      control = 4;
    }
  }
  
  if (control == 4){
    PD_sens = getPTdioda(Turbidity_pin);
    motor_dc(1, 37); 
    Serial.print("NTU :"); 
    Serial.print(PD_sens, 1); 
    Serial.println(" air"); 
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(" Motor Menyala ");
    lcd.setCursor(0,1); 
    lcd.print(" NTU = ");
    lcd.setCursor(9,1); 
    lcd.print(PD_sens, 1);
    unsigned long waktu_isi_sekarang = millis();
    
    if (waktu_isi_sekarang - waktu_isi_awal >= 15000){ //berputar selama 4 detik
      motor_dc(1, 0);
      control = 5; 
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.print(" Motor Mati ");
      lcd.setCursor(0,1); 
      lcd.print(" NTU = ");
      lcd.setCursor(9,1); 
      lcd.print(PD_sens, 1);
      delay(1000);
    }
  }
  
  if (control == 5){
     if (PD_sens <= 2000){
        motor_pompa(1, 255);
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("AIR SUDAH JERNIH");
        lcd.setCursor(0,1); 
        lcd.print(" NTU = ");
        lcd.setCursor(9,1); 
        lcd.print(PD_sens, 1);
        control = 6;
        delay(5000);
        motor_pompa(1, 0);
        lcd.clear();
        Serial.println("POMPA MATI");
        lcd.setCursor(0,0);
        lcd.print(" POMPA MATI ");
        lcd.setCursor(0,1); 
        lcd.print(" NTU = ");
        lcd.setCursor(9,1); 
        lcd.print(PD_sens, 1);
        delay(1000);
      } else if(PD_sens > 2000){
        motor_pompa(1, 255);
        lcd.clear();
        Serial.println("POMPA MENYALA");
        lcd.setCursor(0,0); 
        lcd.print(" POMPA NYALA ");
        lcd.setCursor(0,1);
        lcd.print(" NTU = ");
        lcd.setCursor(9,1); 
        lcd.print(PD_sens, 1);
        delay(5000);
        motor_pompa(1, 0);
        control = 3;
        lcd.clear();
        Serial.println("POMPA MATI");
        lcd.setCursor(0,0); 
        lcd.print(" POMPA MATI ");
        lcd.setCursor(0,1); 
        lcd.print(" NTU = ");
        lcd.setCursor(9,1);
        lcd.print(PD_sens, 1);
        delay(1000);
      }
  }
  
  if (control == 6){
    digitalWrite(Valve_in, HIGH);
    loadcel();
    float mode_berat = berat - berat_awal;
    if (menu == 1){
      lcd.setCursor(0,0);
      lcd.print("Mode krg matang"); Serial.println("Mode krg Matang");
      float kurang_matang = (((100 / 100) * berat_awal ) + berat_awal);
      if (mode_berat >= kurang_matang){
        digitalWrite(Valve_in, LOW);
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print(" Pengisian Air ");
        lcd.setCursor(0,1); 
        lcd.print("   Selesai ");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Takaran Siap");
        lcd.setCursor(0,1);
        lcd.print("  Selesai ");
        control = 0;
        delay(1000);
      }
    }
    
    if (menu == 2){
      lcd.setCursor(0,0);
      lcd.print("Mode Matang"); 
      Serial.println("Mode Matang");
      float matang = (((150 / 100) * berat_awal ) + berat_awal);
      if (mode_berat >= matang){
        digitalWrite(Valve_in, LOW);
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print(" Pengisian Air ");
        lcd.setCursor(0,1); 
        lcd.print("   Selesai ");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("Takaran Siap");
        lcd.setCursor(0,1); 
        lcd.print("  Selesai ");
        control = 0;
        delay(1000);
      }
    }
    
    if (menu == 3){
      lcd.setCursor(0,0);
      lcd.print("Mode trll Matang"); 
      Serial.println("Mode trll Matang");
      float trll_matang = (((300 / 100) * berat_awal ) + berat_awal);
      if (mode_berat >= trll_matang){
        digitalWrite(Valve_in, LOW);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" Pengisian Air ");
        lcd.setCursor(0,1); 
        lcd.print("   Selesai ");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print("Takaran Siap");
        lcd.setCursor(0,1); 
        lcd.print("  Selesai ");
        control = 0;
        delay(1000);
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
void loadcel(){
  Serial.print("Berat:");
  scale.power_up();
  berat = scale.get_units(25);
  if (berat <= 0.0){
    berat = 0.0; 
    Serial.print(berat, 1); 
    Serial.println(" g");
    lcd.setCursor(0,1); 
    lcd.print(" Berat= ");
    lcd.setCursor(9,1); 
    lcd.print(berat, 1);
    lcd.print(" g");
  }
  if (berat < 1000){
    Serial.print(berat, 1); 
    Serial.println(" g");
    lcd.setCursor(0,1); 
    lcd.print(" Berat= ");
    lcd.setCursor(9,1); 
    lcd.print(berat, 1);
    lcd.print(" g");
  }
  if (berat >= 1000){
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
}
float getPTdioda(unsigned int ntu_pin){
    volt = 0;
    for(int i=0; i<800; i++){
        volt += ((float)analogRead(ntu_pin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
    if(volt < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(volt)+5742.3*volt-4353.8; 
    }
//    Serial.print("Kekeruhan air: "); Serial.print(ntu);
//    Serial.print("Voltase sensor: "); Serial.println(volt);
  return ntu;
}
void motor_dc(bool arah, uint8_t pwm_motor){
  analogWrite(pin_pwm_motor, pwm_motor);
  switch(arah){
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
void motor_pompa(bool arah_pompa, uint8_t pwm_pompa){
  analogWrite(pin_pwm_pompa, pwm_pompa);
  switch(arah_pompa){
    case 0:
      digitalWrite(pin_pompa_a, HIGH);
      digitalWrite(pin_pompa_b, LOW);
      break;
    case 1:
      digitalWrite(pin_pompa_a, LOW);
      digitalWrite(pin_pompa_b, HIGH);
      break;
  }
}
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
void PID(){
  if(millis() > time_pid + period_pid){
    time_pid = millis();
    eror_kec = serpoint_berat - berat; 
    PID_P = KP * eror_kec;
    float kec_diference = eror_kec - previous_eror; 
    PID_D = KD * ((eror_kec - previous_eror)/period_pid);
    if(-5 < eror_kec && eror_kec< 5){
      PID_I = PID_I + (KI * eror_kec);
    } else {
      PID_TOTAL = PID_P + PID_I + PID_D;
      if(PID_TOTAL < 20){PID_TOTAL = 20;}
      if(PID_TOTAL > 700){PID_TOTAL = 700;}
      //analogWrite(MOTOR, PID_TOTAL);
      previous_eror = eror_kec;
    }
  }
}
//-----------------------------Menu
ISR(PCINT0_vect){
  clk_State =   (PINB & B00000001); //pin8 interupt
  dt_State  =   (PINB & B00000010); //pin9 interupt
  if (clk_State != Last_State){     
     if (dt_State != clk_State) { 
       counter ++;
     }
     else {
       counter --;
     } 
   } 
   Last_State = clk_State;
}
