#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 22
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

/*********
* modified (a fair bit from):
* 
*  Rui Santos
*  Complete project details at https://randomnerdtutorials.com  
*********


In arduino IDE I use ESP 32 use  DOI IT ESP32 DEVKIT V1 board
*/

// Load Wi-Fi library
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>


#define SEALEVELPRESSURE_HPA (1013.25)

#define WEATHER_CHANGE_DUE_TO_PRESSURE_CHANGE_HPA (3.5f)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// don't check in WifiConfig.h , add to git ignore
#include "WifiConfig.h"
//const char* ssid     = "";
//const char* password = "";


// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;


struct bme_measurements_struct
{
  float temp_C;
  float temp_F;
  float humidity_Percent;
  float pressure_Hpa;
  float altitude;
  float lastMeasurePressure; 
  bool stormWarning;
} sensorData;

int measureCount = 0;

void setup() {
  Serial.begin(115200);
  bool status;
  measureCount = 0;
  sensorData.temp_C = 0;
  sensorData.temp_F = 0;
  sensorData.humidity_Percent = 0;
  sensorData.pressure_Hpa = 0;
  sensorData.altitude = 0;  
  sensorData.lastMeasurePressure = 0;
  sensorData.stormWarning = false;
    
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  if (!bme.begin(0x76)) {
    //Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Connect to Wi-Fi network with SSID and password
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    //Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  
}

void loop()
{
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) 
  {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() &&
           currentTime - previousTime <= timeoutTime &&
           WiFi.status() == WL_CONNECTED) 
    {  // loop while the client's connected
      currentTime = millis();

      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            sensorData.temp_C = bme.readTemperature();      
            sensorData.temp_F = 1.8 * sensorData.temp_C + 32;      
            sensorData.humidity_Percent = bme.readHumidity();       
            sensorData.pressure_Hpa = bme.readPressure() / 100.0f;
            sensorData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

                  
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the table 
            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial;}");
            client.println("table { border-collapse: collapse; width:35%; margin-left:auto; margin-right:auto; }");
            client.println("th { padding: 12px; background-color: #0043af; color: white; }");
            client.println("tr { border: 1px solid #ddd; padding: 12px; }");
            client.println("tr:hover { background-color: #bcbcbc; }");
            client.println("td { border: none; padding: 12px; }");
            client.println(".sensor { color:white; font-weight: bold; background-color: #bcbcbc; padding: 1px; }");
            
            // Web Page Heading
            client.println("</style></head><body><h1>Adrian's weather!!");
            client.println("<table><tr><th>Sensor Values</th></tr>");
                     
            client.println("<tr><td>Temp. Celsius</td><td><span class=\"sensor\">");
            client.println(sensorData.temp_C);
            client.println(" *C</span></td></tr>");  
            
            client.println("<tr><td>Pressure</td><td><span class=\"sensor\">");
            client.println(sensorData.pressure_Hpa);
            client.println(" hPa</span></td></tr>");
                      
            client.println("<tr><td>Humidity</td><td><span class=\"sensor\">");
            client.println(sensorData.humidity_Percent);            
            client.println(" %</span></td></tr>"); 

            //client.println("<tr><td>Approx. Altitude</td><td><span class=\"sensor\">");
            //client.println(sensorData.altitude);
            //client.println(" m</span></td></tr>"); 
            //client.println("<tr><td>Temp. Fahrenheit</td><td><span class=\"sensor\">");
            //client.println(sensorData.temp_F);
            //client.println(" *F</span></td></tr>");       

            /// these values done by measure at our location and checking weather , so >993 was good summer weather
            client.println("<tr><td>Weather likely to be</td><td><span class=\"sensor\">");
            if (sensorData.pressure_Hpa > 993) client.println("Sunny");
            else if (sensorData.pressure_Hpa < 990) client.println("Wet");
            else client.println("Fair");

            if (sensorData.stormWarning == true)
            {
               client.println("<tr><td>Weather warn</td><td><span class=\"sensor\">");
               client.println("Weather Warning STORM");
               client.println("</span></td></tr>");                     
            }
                       
            client.println(" *F</span></td></tr>");                  
            
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();

            if (measureCount++ > 50)
            {
              measureCount = 0;
              // this condition means the pressure has dropped by more than the amount you would expect
              if ( fabs(sensorData.pressure_Hpa) - fabs(sensorData.lastMeasurePressure) > WEATHER_CHANGE_DUE_TO_PRESSURE_CHANGE_HPA)
              {                
                sensorData.stormWarning = true;         
              }
              else
              {
                sensorData.stormWarning = false;         
              }

              sensorData.lastMeasurePressure = sensorData.pressure_Hpa;
            }
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }      
      }// if new client    
    } //while connected
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();    
    Serial.println("Client disconnected.");
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) //wifi broken attempt reconnect
    { 
      server.stop(); // probably assume this has already stopped??
      sleep(10);   
      // Connect to Wi-Fi network with SSID and password
      Serial.print("attempting to reconnect to ");
      Serial.println(ssid);
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      // Print local IP address and start web server
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      server.begin();
    }    
  }
}
