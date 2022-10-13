
#include <Wire.h>
#define THERMISTORPIN 2        
// resistance of termistor at 25 degrees C
#define THERMISTORNOMINAL 10000      
#define TEMPERATURENOMINAL 25   
// Accuracy - Higher number is bigger
#define NUMSAMPLES 10
// Beta coefficient from datasheet
#define BCOEFFICIENT 3950
// the value of the R1 resistor
#define SERIESRESISTOR 10000    
//prepare pole 
#include <HardwareSerial.h>
#include <ArduinoJson.h>
uint16_t samples[NUMSAMPLES];

const int xpin= 34;
const int ypin = 35;
const int zpin = 32;
const int xpini = 33;
const int ypini = 25;
const int zpini = 26;
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
//////////////////////////////////////////////////////////////////
#define ADC_ref 2.56
 
#define zero_x 1.569
 
#define zero_y 1.569
 
#define zero_z 1.569
 
#define sensitivity_x 0.3
 
#define sensitivity_y 0.3
 
#define sensitivity_z 0.3
 
unsigned int value_x;
unsigned int value_y;
unsigned int value_z;
 
float xv;
float yv;
float zv;
 
float angle_x;
float angle_y;
float angle_z;



int pos = 0;  



//const int  = 0;
int EMGVal = 0;
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
     
   ///////////////////////////////######################TEMPERATURE NTC######################//////////////////////////////////////////
  uint8_t i;
  float average;
 
 // saving values from input to pole
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
  
  }
 
   // average value of input
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  //resistance
  average = 4095 / average - 1;
  average = SERIESRESISTOR / average;

 //resistence to temperature
  float temperature;
  temperature = average / THERMISTORNOMINAL; 
  temperature = log(temperature);   
  temperature /= BCOEFFICIENT;                  
  temperature += 1.0 / (TEMPERATURENOMINAL + 273.15); 
  temperature = 1.0 / temperature;                 
  temperature -= 273.15;                         // convert to C
 
  Serial.print("Temperature: "); 
  Serial.print(temperature);
  Serial.println("°C");      

  //////////////////////////////////////////////###########################LEG NODES###################//////////////////////////////////////
  
   Serial.println("1");

   float w = wroom(xpin,ypin,zpin);
    Serial.println(w);
  Serial.println("2");
    float u=wroom(xpini,ypini,zpini);
    Serial.println(u);
  
 
   ///////////////////////////////////////////////////################STEP COUNTING#######################//////////////////////////////////////////
   if ( ypin <= 450*4 && xpin>=390*4)
{ count++;
//MySerial.print(count);


}
if (ypin >= 460*4 && xpin<=370*4)
{
count++;
// MySerial.print(count);

}

float distance = count * 0.0006; // My step it is on average 60cm. 
Serial.println("steps:");
Serial.println(count);
Serial.println("distance");
Serial.println(distance);

  //delay(1000);
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
    EMGVal = analogRead(EMGPin);
  Serial.println(EMGVal);


   x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);     //Formula to calculate x values in degree
     Serial.print("Angle");
     Serial.print(x); 
     ////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
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
  
  //sensorVoltage = EMGVal*(3.0/4095);
  //Serial.println(sensorVoltage);
 
 
  
 
  
  /////////////////////////////////////////////////////////####################SENDER DATA###################////////////////////////////////
 //
 StaticJsonDocument<1000> doc;

  doc["Temp"]= temperature;
  doc["pressure_point_1"]= l;
  doc["pressure_point_2"]= j;
  doc["pressure_point_3"]= k;
  doc["steps"]=count;
  doc["distance"]=distance;
  doc["EMG"]=EMGVal;
  doc["angle"]=x;
  doc["AC1"]=w;
  doc["AC2"]=u;
  //doc["AC3"]=v;
  

  serializeJson(doc, SerialPort);                                // Send it to Sender serial port
  delay(5200);

}
float wroom(int x,int y,int z)
{
  value_x = analogRead(x);
value_y = analogRead(y);
value_z = analogRead(z);
 
xv=(value_x/4095.0*ADC_ref-zero_x)/sensitivity_x;
 
//Serial.print ("x= ");
//Serial.print (xv);
//Serial.print(" g ");
 
yv=(value_y/4095.0*ADC_ref-zero_y)/sensitivity_y;
 
//Serial.print ("y= ");
//Serial.print (yv);
//Serial.print(" g ");
 
zv=(value_z/4095.0*ADC_ref-zero_z)/sensitivity_z;
 
//Serial.print ("z= ");
//Serial.print (zv);
//Serial.print(" g ");
 
//Serial.print("\n");
 
//Serial.print("Rotation ");
 
//Serial.print("x= ");
 
angle_x =atan2(-yv,-zv)*57.2957795+180;
 
Serial.print(angle_x);
//Serial.print(" deg");
//Serial.print(" ");
 
//Serial.print("y= ");
/*
angle_y =atan2(-xv,-zv)*57.2957795+180;
 
Serial.print(angle_y);
Serial.print(" deg");
Serial.print(" ");
 
Serial.print("z= ");
 
angle_z =atan2(-yv,-xv)*57.2957795+180;
 
Serial.print(angle_z);
Serial.print(" deg");
Serial.print("\n");
 
delay(1000);*/
  return value_x;
  
}
float pressure(float b)
{
   int fsrADC = analogRead(b);
   float force;
  
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    
    float fsrV = fsrADC * VCC / 4095.0;

    float fsrR = R_DIVC * (VCC / fsrV - 1.0);
    //Serial.println("Resistance: " + String(fsrR) + " ohms");
   
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
  return fsrADC ;
  
}
