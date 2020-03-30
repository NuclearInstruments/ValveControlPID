#include <Wire.h>


#define DAC1 A0 //25
#define VALVE_OUT_PIN A1
#define DAC_CONV ((255.0*6)/6)/3.3

#define N_PRESSURE_SENSORS 1
#define SIMULATE_SENSORS 0

typedef struct
{
  int32_t C[6];
} t_5525DSO_calibration_table;



void CalibrateDate_5525DSO(t_5525DSO_calibration_table CT, int32_t raw_temp, int32_t raw_pressure, float *T, float *P);
bool Convert_5525DSO(int address, int32_t *temp, int32_t *pressure);
bool Reset_5525DSO(int address);
bool ReadCalibration_5525DSO(int address, t_5525DSO_calibration_table *ct);
bool FirstConversion_5525DSO(int address);
int read_pressure_sensor(int idx, float *T, float *P);

uint8_t pressure_sensor_i2c_address [] = {0x76};
t_5525DSO_calibration_table PRES_SENS_CT[N_PRESSURE_SENSORS];


void setup() {

  pinMode (VALVE_OUT_PIN, OUTPUT);

  digitalWrite(VALVE_OUT_PIN, LOW);
          
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  __service_i2c_detect();


  
  for (int j=0;j<N_PRESSURE_SENSORS;j++)
  {
    ReadCalibration_5525DSO( (int) pressure_sensor_i2c_address [j], &PRES_SENS_CT[j]);
    Serial.print("SENSOR:           ");   Serial.println(j);
    Serial.print("SENS_T1:          ");   Serial.println(PRES_SENS_CT[j].C[0]);
    Serial.print("OFF_T1:           ");   Serial.println(PRES_SENS_CT[j].C[1]);
    Serial.print("TCS:              ");   Serial.println(PRES_SENS_CT[j].C[2]);
    Serial.print("TCO:              ");   Serial.println(PRES_SENS_CT[j].C[3]);
    Serial.print("TREF:             ");   Serial.println(PRES_SENS_CT[j].C[4]);
    Serial.print("TEMPSENS:         ");   Serial.println(PRES_SENS_CT[j].C[5]);
  }
}

  long int timeosc =0;
  
void loop() {

  static float pid_error=0;
  static float pid_integral=0;
  static float pid_prec=0;
  static float pid_out=0;
  static bool state = false;
  static long int breathing_cycle_count = 0;
  static float previous_best_pid_out = 0;
  static float overshoot_of_previous_best_pid_out = 1.1; /*just a guess, will need to be tuned.
                                                          I imagine the system will like the valve to be initially
                                                          slightly more open in the beggining of the cycle*/
  static float new_cycle_dac_set_val; 
  static long int initial_open_wait_ms = 300;

  static float Pset = 0;
  float Pmeas = 0;
  float Tmeas = 0;

  float PID_P = 60;
  float PID_I = 1;
  float PID_D = 70;  

  if (millis()-timeosc > 5000)
  {
    timeosc=millis();
    if (state == false)
    {
      Pset=20;
      if (breathing_cycle_count > 0)
      {
        new_cycle_dac_set_val = overshoot_of_previous_best_pid_out * previous_best_pid_out;
        dacWrite(DAC1, new_cycle_dac_set_val);
        delay(initial_open_wait_ms); /*this will for sure need to be optimized to some more optimal value 
                     giving the system enough time (and not too long) to react before the first pressure sensor read below*/ 
      }
    }
    else
    {
       Pset=0;
    }
    state = !state;
  }
  
  read_pressure_sensor(0, &Tmeas, &Pmeas);


 
  pid_error = Pset-Pmeas;
  pid_integral += pid_error ;
  if ((pid_integral*PID_I) > 255 ) pid_integral = (255/PID_I);
  if ((pid_integral*PID_I) <-255 ) pid_integral = -(255/PID_I);
  
  pid_out= PID_P* pid_error + PID_I*pid_integral + PID_D*(pid_error-pid_prec);
  if (pid_out<0) pid_out=0;
  if (pid_out>240) pid_out=240;

   pid_prec = pid_error;
  
  dacWrite(DAC1, pid_out);
  if (state == true) /*keep recording the latest pid_out value in the inhalation cycle
                     last one should be the "best" for the next inhalation cycle*/
  {
    previous_best_pid_out = pid_out;
  }
  breathing_cycle_count++;
  
  Serial.println(String(Pset) + "," + String(Pmeas) + "," + String(pid_out) + "," + String(0) + "," + String(pid_error));
 
  /*for (int i=0;i<DAC_CONV;i++){
    dacWrite(DAC1, i);
    Serial.println("UP " + String(i));
    delay(20);
  }
delay(2000);

  for (int i=DAC_CONV-1;i>=0;i--){
    dacWrite(DAC1, i);
    delay(20);
     dacWrite(DAC1, i);
    Serial.println("DOWN " + String(i));
  }  */


}



void __service_i2c_detect()
{
   byte error, address;
  int nDevices;
  Serial.println("Scanning... I2C");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }

}


int read_pressure_sensor(int idx, float *T, float *P)
{
  if (idx < N_PRESSURE_SENSORS)
  {

    if (SIMULATE_SENSORS==1)
    {
      
    }
    else
    {
      uint8_t i2c_address = pressure_sensor_i2c_address[idx];
      int32_t raw_temp;
      int32_t raw_pressure;

      
      if (Convert_5525DSO((int)i2c_address, &raw_temp, &raw_pressure))
      {
        CalibrateDate_5525DSO(PRES_SENS_CT[idx], raw_temp, raw_pressure, T, P);


      }
    }

    return 0;
  }
  else
    return -1;
}



bool ReadCalibration_5525DSO(int address, t_5525DSO_calibration_table *ct)
{
  bool bres=true;
  int error;
  for (int i=0;i<6;i++)
  {
    Wire.beginTransmission(address);
    Wire.write(0xA0 + ((i+1)<<1)); // MSB 
    error = Wire.endTransmission(); 
    Wire.requestFrom(address,2);
    byte MSB = Wire.read();
    byte LSB = Wire.read();
    error = Wire.endTransmission(); 
    ct->C[i] = (MSB<<8)+LSB;
  }

  return bres;
}

bool Reset_5525DSO(int address)
{
  int error;
    Wire.beginTransmission(address);
    Wire.write(0x1E); // MSB 
    error = Wire.endTransmission(); 
  return true;
}

bool FirstConversion_5525DSO(int address)
{
    Wire.beginTransmission(address);
    Wire.write(0x58); // MSB 
    Wire.endTransmission(); 
    return true;
}

bool Convert_5525DSO(int address, int32_t *temp, int32_t *pressure)
{
  static int32_t last_temp;
  static int32_t last_pressure;
  static uint8_t temp_read=0;
  bool bres=true;
  int error;
  byte b1, b2, b3;


    Wire.beginTransmission(address);
    Wire.write(0x48); // MSB 
    error = Wire.endTransmission(); 

    delay(15);
    
    Wire.beginTransmission(address);
    Wire.write(0x00); // MSB 
    error = Wire.endTransmission(); 


    delay(1);
    
    error = Wire.requestFrom(address,3, true);
    if (error < 3) return false;
    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();

    *pressure = (b1<<16) + (b2<<8) + b3;

    if (temp_read==0)
    {
      
      Wire.beginTransmission(address);
      Wire.write(0x58); // MSB 
      error = Wire.endTransmission(); 
  
      delay(15);
      
      Wire.beginTransmission(address);
      Wire.write(0x00); // MSB 
      error = Wire.endTransmission(); 
  
      delay(1);
      
      error = Wire.requestFrom(address,3, true);
      if (error < 3) return false;
      b1 = Wire.read();
      b2 = Wire.read();
      b3 = Wire.read();
      *temp = (b1<<16) + (b2<<8) + b3;
      last_temp= *temp;
      temp_read=50;
    }
    else
    {
      temp_read--;
      *temp = last_temp;
    }
   
  return true;
}

/*
bool Convert_5525DSO(int address, int32_t *temp, int32_t *pressure)
{
  static int32_t last_temp;
  static int32_t last_pressure;
  static uint8_t temp_read=1;
  bool bres=true;
  int error;
  byte b1, b2, b3;
  
  Wire.beginTransmission(address);
    Wire.write(0x48); // MSB 
    error = Wire.endTransmission(); 

    delay(15);
    
    Wire.beginTransmission(address);
    Wire.write(0x00); // MSB 
    error = Wire.endTransmission(); 


    delay(1);
    
    error = Wire.requestFrom(address,3, true);
    if (error < 3) return false;
    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();

    *pressure = (b1<<16) + (b2<<8) + b3;
     
      
    Wire.beginTransmission(address);
    Wire.write(0x58); // MSB 
    error = Wire.endTransmission(); 

    delay(15);
    
    Wire.beginTransmission(address);
    Wire.write(0x00); // MSB 
    error = Wire.endTransmission(); 

    delay(1);
    
    error = Wire.requestFrom(address,3, true);
    if (error < 3) return false;
    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();
    *temp = (b1<<16) + (b2<<8) + b3;
  
   
  return true;
}

*/
void CalibrateDate_5525DSO(t_5525DSO_calibration_table CT, int32_t raw_temp, int32_t raw_pressure, float *T, float *P)
{
  int32_t Q1 = 15;
  int32_t Q2 = 17;
  int32_t Q3 = 7;
  int32_t Q4 = 5;
  int32_t Q5 = 7;
  int32_t Q6 = 21;

  int32_t C1 = CT.C[0];
  int32_t C2 = CT.C[1];
  int32_t C3 = CT.C[2];
  int32_t C4 = CT.C[3];
  int32_t C5 = CT.C[4];
  int32_t C6 = CT.C[5];
    
  int32_t dT;
  int64_t OFF;
  int64_t SENS;

  int32_t Temp;
  int64_t Pres;
  //Serial.println(raw_temp);
  //Serial.println(raw_pressure);
  dT = raw_temp - (C5 * pow(2,Q5));
  Temp = 2000 + ( (dT*C6) /pow(2,Q6));
  OFF = (C2 * pow(2,Q2)) + ((C4 * dT)/(pow(2,Q4)));
  SENS = (C1 * pow(2,Q1)) + ((C3 * dT)/(pow(2,Q3)));
  Pres = (((raw_pressure * SENS)/(pow(2,21))) - OFF)/(pow(2,15));

  *T = ((float)Temp)/100.0;
  *P = ((float)Pres)/10000.0 * 68.9476;
  

}
