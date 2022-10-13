

#include "WiFi.h"
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

const char* ssid = "Redmi";    // Enter SSID here
const char* password = "san2003.4.1";  //Enter Password here

String serverName  = "https://exploremychoice.in";
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;
HardwareSerial SerialPort(2); // use UART2




void setup() 
{
  Serial.begin(9600);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
   delay(3000);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
  
}

void loop() 
{
  StaticJsonDocument<1000> doc;
  DeserializationError error = deserializeJson(doc, SerialPort);

  if(error){

  Serial.println("Error");
  }

  Serial.println("Recieved data:");
  Serial.println("temperature");
  float temp=doc["Temp"];
  Serial.println(temp);
  Serial.println("pressure 1");
  float pressure_point_1=doc["pressure_point_1"];
  Serial.println(pressure_point_1);
  Serial.println("pressure 2");
  float pressure_point_2=doc["pressure_point_2"];
  Serial.println(pressure_point_2);
  Serial.println("pressure 2");
  float pressure_point_3=doc["pressure_point_3"];
  Serial.println(pressure_point_3);
  Serial.println("steps");
  int steps=doc["steps"];
  Serial.println(steps);
  Serial.println("distance");
  float distance=doc["distance"];
  Serial.println(distance);
  Serial.println("EMG");
  int EMG=doc["EMG"];
  Serial.println(EMG);
  Serial.println("angle");;
  double angle=doc["angle"];
  Serial.println(angle);
  Serial.println("acelerometer 1");
  float AC1=doc["AC1"];
  Serial.println(AC1);
  Serial.println("acelerometer 2");
  float AC2=doc["AC2"];
  Serial.println(AC2);
  Serial.println("acelerometer 2");
  float AC3=doc["AC3"];
  Serial.println(AC3);
  
  
  
//////////////////////////////////////////#####################################################################################33
   if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;

      String serverPath = serverName + "/sih/ortho-mobi/senddataright.php";
       Serial.println(serverPath);
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath);
      http.addHeader("Content-Type","application/x-www-form-urlencoded");
     
       int httpResponseCode = http.POST("emg_rate="+String(EMG)+"&pressure_point_1="+String(pressure_point_1)+"&pressure_point_2="+String(pressure_point_2)+"&pressure_point_3="+String(pressure_point_3));
      
      
      
      // Send HTTP GET request
      
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
     delay(5000);

}
