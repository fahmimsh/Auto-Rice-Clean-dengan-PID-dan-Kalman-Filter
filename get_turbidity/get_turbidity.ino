#include <FahmiKalmanFilter.h>
#define Turbidity_pin PB9
float volt, ntu;
float ntu_filter;

 /*FahmiKalmanFilter(e_mea, e_est, q);
    e_mea: Measurement Uncertainty 
    e_est: Estimation Uncertainty 
    q: Process Noise */
    
FahmiKalmanFilter Kalman_filter(2, 2, 0.01);
//serial output refresh time pada ambildata
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
    volt = 0;
    for(int i=0; i<800; i++){
        volt += ((float)analogRead(Turbidity_pin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
    if(volt < 0.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(volt)+5742.3*volt-4353.8;
      //rumus kalmanfilter pada library yang sudah dibuat
      ntu_filter = Kalman_filter.updateEstimate(ntu);
    }
    //untuk mengambil 100 data untuk di filter maka perlu pengambilan 100ms sekali
    if (millis() > refresh_time){
      Serial.print("Volt: "); 
      Serial.print(volt);
      Serial.print(" || NTU : "); 
      Serial.print(ntu);
      Serial.print(" || NTU Filter : "); 
      Serial.println(ntu);  
      refresh_time = millis()+ SERIAL_REFRESH_TIME;
    }
    
  return ntu_filter;
}
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
