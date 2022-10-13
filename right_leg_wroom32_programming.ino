
#include <Wire.h> 
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#define NUMSAMPLES 10
uint16_t samples[NUMSAMPLES];

const int xpin= 34;
const int ypin = 35;
const int zpin = 32;
const int xpini = 33;
const int ypini = 25;
const int zpini = 26;
const int xpine = 27;
const int ypine= 14;
const int zpine = 12;
const int FSR_PIN = 13;
const int FSR_PIN1 = 36;
const int FSR_PIN2= 39;
const int EMGPin = 15;

int count =0;

HardwareSerial SerialPort(2); // use UART2

const int MPU_addr=0x68;         //I2C MPU6050 Address

int16_t axis_X,axis_Y,axis_Z;    

int minVal=265;

int maxVal=402;

double x;

double y;

double z;



int pos = 0;  



//const int  = 0;
int EMGVal1 = 0;
//const float R_DIV = 47000.0;  // resistor used to create a voltage divider
//const float flatResistance = 25000.0; // resistance when flat
//const float bendResistance = 100000.0;
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIVC = 3230.0;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
  Wire.begin();                        //Begins I2C communication

  Wire.beginTransmission(MPU_addr);    //Begins Transmission with MPU6050

  Wire.write(0x6B);                    //Puts MPU6050 in Sleep Mode

  Wire.write(0);                       //Puts MPU6050 in power mode 

  Wire.endTransmission(true);

}

void loop() {
     
 

  //////////////////////////////////////////////###########################LEG NODES###################/////////////////////////////////////
   
   Serial.println("1");  

   float w = wroom(xpin,ypin,zpin);
    Serial.println(w);
  Serial.println("2");
    float u=wroom(xpini,ypini,zpini);
    Serial.println(u);
  Serial.println("3");
   float v=wroom(xpine,ypine,zpine);
    Serial.println(v);
 




//////////////////////////////////////////////////####################FOOT PRESSURE##############/////////////////////////////////////////////////////
  Serial.println("1");
   float l =pressure(FSR_PIN);
   Serial.println(l);
  Serial.println("2");
   float j=pressure(FSR_PIN1);
    Serial.println(j);
  Serial.println("3");
   float k=pressure(FSR_PIN2);
    Serial.println(k);
  /////////////////////////////////////////////////////#################ANGLE OF LEG#####################/////////////////////////////////////////////
  Wire.beginTransmission(MPU_addr); //Begins I2C transmission 

  Wire.write(0x3B);                 //Start with register 0x3B (ACCEL_XOUT_H)             

  Wire.endTransmission(false);

  Wire.requestFrom(MPU_addr,14,true); //Request 14 Registers from MPU6050

  

  axis_X=Wire.read()<<8|Wire.read(); //Obtain 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 

  axis_Y=Wire.read()<<8|Wire.read(); //0x3B (ACCEL_YOUT_H) & 0x3C (ACCEL_YOUT_L)

  axis_Z=Wire.read()<<8|Wire.read(); //0x3B (ACCEL_ZOUT_H) & 0x3C (ACCEL_ZOUT_L)

    

    //int xAng = map(axis_X,minVal,maxVal,-90,90); 

    int yAng = map(axis_Y,minVal,maxVal,-90,90);

    int zAng = map(axis_Z,minVal,maxVal,-90,90);


   x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);     //Formula to calculate x values in degree
     Serial.print("Angle");
     Serial.print(x); 
  
  //int ADCflex = analogRead(flux);
    //float Vflex = ADCflex * 3.3 / 4095.0;
    //float Rflex = R_DIV * (3.3 / Vflex - 1.0);
    //Serial.println("Resistance: " + String(Rflex) + " ohms");

    // Use the calculated resistance to estimate the sensor's bend angle:
    //float angle = map(Rflex, flatResistance, bendResistance, 0, 90.0);
    //Serial.println("Bend: " + String(angle) + " degrees");
    //Serial.println();
  //EMG
  //float sensorVoltage;
  ////////////////////////////////////////##################EMG INPUT######################////////////////////////////////////////////
  EMGVal1 = analogRead(EMGPin);
  Serial.println(EMGVal1);
  //sensorVoltage = EMGVal*(9/4095);
  //Serial.println(sensorVoltage);
 
 
  
 
  
  /////////////////////////////////////////////////////////####################SENDER DATA###################////////////////////////////////
 //
 StaticJsonDocument<1000> doc;

  
  doc["pressure_point_4"]= l;
  doc["pressure_point_5"]= j;
  doc["pressure_point_6"]= k;
  doc["EMG1"]=EMGVal1;
  doc["angle1"]=x;
  doc["AC4"]=w;
  doc["AC5"]=u;
  doc["AC6"]=v;
  

  serializeJson(doc, SerialPort);                                // Send it to Sender serial port
  delay(5200);

}
float wroom(int x,int y,int z)
{
  int xvalue;
 int yvalue;
 int zvalue;
  double x_g_value, y_g_value, z_g_value;
  double roll, pitch, yaw;
  xvalue = analogRead(x);                              //reads values from x-pin & measures acceleration in X direction 
  int a = map(xvalue, 267, 400, -100, 100);               //maps the extreme ends analog values from -100 to 100 for our understanding
// you need to replace the 267 & 400 value with your values from calibration
  float xg = (float)a/(-100.00);                          //converts the mapped value into acceleration in terms of "g"
  //Serial.print(xg);                                       //prints value of acceleration in X direction
  //Serial.print("g   ");                                   //prints "g"
   
  yvalue = analogRead(y);
  int b = map(yvalue, 272, 406, -100, 100);
  float yg = (float)b/(-100.00);
  //Serial.print("\t");
  //Serial.print(yg);
  //Serial.print("g   "); 
 
  zvalue = analogRead(z);
  int c = map(zvalue, 277, 410, -100, 100);
  float zg = (float)c/(100.00);
  //Serial.print("\t"); 
  //Serial.print(zg);
  //Serial.println("g   ");
  delay(100);
   x_g_value = ( ( ( (double)(xg * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
  y_g_value = ( ( ( (double)(yg * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
  z_g_value = ( ( ( (double)(zg* 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */ 

  roll = ( ( (atan2(y_g_value,z_g_value) * 180) / 3.14 ) + 180 ); /* Formula for roll */
  pitch = ( ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ) + 180 ); /* Formula for pitch */
  yaw = ( ( (atan2(x_g_value,y_g_value) * 180) / 3.14 ) + 180 ); /* Formula for yaw */
  /* Not possible to measure yaw using accelerometer. Gyroscope must be used if yaw is also required */

  //Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print("\t");
  //Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\n\n");
  //Serial.print("yaw = ");
  Serial.print(yaw);
  Serial.print("\n\n");
 
  return xvalue;
  
}
float pressure(float b)
{
   int fsrADC = analogRead(b);
  
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    
    float fsrV = fsrADC * VCC / 4095.0;

    float fsrR = R_DIVC * (VCC / fsrV - 1.0);
    //Serial.println("Resistance: " + String(fsrR) + " ohms");
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force = (((fsrG - 0.00075) / 0.00000032639)*98.0665)/1000;
     
    else
      force =  ((fsrG / 0.000000642857)*98.0665)/1000;
     
    //Serial.println("Force: " + String(force) + " pa");
    Serial.println(force); 
   
    
  }
  else
  {
    //No pressure detected
  }
  return fsrADC;
  
}
