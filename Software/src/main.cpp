#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define DEFAULT_PIN_SS    16          // GPIO16, D0
#define DEFAULT_PIN_DIO0  15          // GPIO15, D8
#define DEFAULT_PIN_RST   NOT_A_PIN   // Unused
#define SFACTOR  10  //spreading factor
static const int RXPin = D3, TXPin = D4;
static const uint32_t GPSBaud = 9600;

uint32_t  freq = 868100000; 					// Channel 0, 868.1 MHz
//uint32_t  freq = 868300000; 					// Channel 1, 868.3 MHz
//uint32_t  freq = 868500000; 					// in Mhz! (868.5)
//uint32_t  freq = 867100000; 					// in Mhz! (867.1)
//uint32_t  freq = 867300000; 					// in Mhz! (867.3)
//uint32_t  freq = 867500000; 					// in Mhz! (867.5)
//uint32_t  freq = 867700000; 					// in Mhz! (867.7)
//uint32_t  freq = 867900000; 					// in Mhz! (867.9)
//uint32_t  freq = 868800000; 					// in Mhz! (868.8)
//uint32_t  freq = 869525000; 					// in Mhz! (869.525)

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

int counter = 0;
double latitude;
double longitude;

long delayTime = 5000;
long lastTime = 0;

boolean gpsStatus = false;

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print(latitude, 6);
    Serial.print(F(" , "));
    Serial.println(longitude, 6);

    gpsStatus = true;
  }
  else
  {
    Serial.print(F("INVALID"));
    Serial.println();

    gpsStatus = false;
  }
}

void setup() 
{
  Serial.begin(9600);
  ss.begin(GPSBaud);

  LoRa.setPins(DEFAULT_PIN_SS, DEFAULT_PIN_RST, DEFAULT_PIN_DIO0);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(freq)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(SFACTOR);           // ranges from 6-12,default 7 see API docs
}

void loop() 
{
  long actualTime = millis();

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if(((actualTime - lastTime) > delayTime) && gpsStatus)
  {
    Serial.print("Sending packet: ");
  
    const size_t capacity = JSON_OBJECT_SIZE(3);
    DynamicJsonDocument data(capacity);
    data["id"] = 101;
    data["lat"] = latitude;
    data["lon"] = longitude;

    String message;
    serializeJson(data, message);
    Serial.println(message);

    // send packet
    unsigned long startTime = millis();
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket(true);
    unsigned long stopTime = millis();

    Serial.print("Send time: ");
    Serial.print(stopTime - startTime);
    Serial.println(" ms");

    counter++;
    lastTime = actualTime;
  }
  //delay(5000);
}