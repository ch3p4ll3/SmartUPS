#include "./config.h"

#include <Arduino.h>

#include <ArduinoJson.h>
#include "EspMQTTClient.h"

#include <WiFiUdp.h>
#include <NTPClient.h>

#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>

#include "INA226.h"


#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif


bool isOutagePresent = false;
bool timeSetted = false;
bool isConnected = false;
bool shutdownFired = false;
unsigned long int timePassedSinceOutage = 0;
unsigned long int timePassed = 0;
float batteryLife = BATTERY_AH;

static int taskCore = 0;

RTC_DS3231 rtc;
DateTime lastOutage;
WiFiUDP wifiUdp;

INA226 INA0(0x40);
NTPClient timeClient(wifiUdp, "at.pool.ntp.org", TIMEZONE_DELTA * 3600, 60000);
EspMQTTClient client(
  WIFI_SSID,
  WIFI_PASSWORD,
  MQTT_BROKER_HOSTNAME,  // MQTT Broker server ip
  MQTT_USERNAME,   // Can be omitted if not needed
  MQTT_PASSWORD,   // Can be omitted if not needed
  MQTT_CLIENT_NAME,     // Client name that uniquely identify your device
  MQTT_PORT              // The MQTT port, default to 1883. this line can be omitted
);


void coreTask( void * pvParameters );
void setTime();
void onPowerOutage();
void onPowerRestored();
void shutdown();
bool CheckIfOutagePresent();


void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
  #endif

  Wire.begin();

  if (!rtc.begin()) {
    DEBUG_PRINTLN("RTC not detected");

    while (1); // Hang indefinitely if RTC is not found
  }

  Wire.begin();
  if (!INA0.begin() )
  {
    DEBUG_PRINTLN("INA0 could not connect. Fix and Reboot");
  }

  INA0.setMaxCurrentShunt(INA226_MAX_CURRENT, INA226_SHUNT_RESISTOR_VALUE, false);
  INA0.setAverage(INA226_4_SAMPLES);

  xTaskCreatePinnedToCore(
    coreTask,   /* Function to implement the task */
    "coreTask", /* Name of the task */
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    0,          /* Priority of the task */
    NULL,       /* Task handle. */
    taskCore);  /* Core where the task should run */

  client.setMaxPacketSize(400);
}

void loop() {
  isOutagePresent = CheckIfOutagePresent();

  if (isOutagePresent && !timeSetted){
    onPowerOutage();
  }
  else if(!isOutagePresent && timeSetted){
    onPowerRestored();
  }

  if (isOutagePresent && !shutdownFired){
    shutdown();
  }

  client.loop();
  delay(10);
}

void onConnectionEstablished() {
  timeClient.begin();
  timeClient.update();
  timeClient.forceUpdate();

  DEBUG_PRINTLN("Connected to MQTT broker");
  isConnected = true;

  setTime();
}

void coreTask( void * pvParameters ){
    while(true){
      if (millis() - timePassed >= DELAY_BETWEEN_STATUS * 1000 && isConnected){
        String toSend;
        StaticJsonDocument<400> doc;

        DateTime now = rtc.now();

        float current = INA0.getCurrent_mA();
        float voltage = INA0.getBusVoltage();

        if (isOutagePresent)
          batteryLife -= (current / 1000) * (DELAY_BETWEEN_STATUS / 3600.0);
        else{
          if (batteryLife > BATTERY_AH)
            batteryLife += (current / 1000) * (DELAY_BETWEEN_STATUS / 3600.0);
          else
            batteryLife = BATTERY_AH;
        }

        doc["lastOutageTime"] = lastOutage.timestamp() + "Z";
        doc["isOutagePresent"] = isOutagePresent;
        doc["time"] = now.timestamp() + "Z";
        doc["voltage"] = voltage;
        doc["current"] = current;
        doc["power"] = INA0.getPower();
        doc["shuntVoltage"] = INA0.getShuntVoltage_mV();
        doc["batteryPercentage"] = (batteryLife / BATTERY_AH) * 100;
        doc["WhLeft"] = batteryLife * voltage;
        doc["AhLeft"] = batteryLife;

        serializeJson(doc, toSend);

        client.publish(STATUS_TOPIC, toSend);

        timePassed = millis();
        DEBUG_PRINTLN("status message sent");
      }

      vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void setTime() {
  timeClient.update();

  rtc.adjust(DateTime(timeClient.getEpochTime()));
}

void onPowerOutage(){
  timeSetted = true;
  timePassedSinceOutage = millis();
  lastOutage = rtc.now();
  DEBUG_PRINTLN("Outage detected");
  client.publish(OUTAGE_TOPIC, "toSend");
}


void onPowerRestored(){
  timeSetted = false;
  shutdownFired = false;
  DEBUG_PRINTLN("Power restored");
  client.publish(POWER_RESTORED_TOPIC, "toSend");
}


void shutdown(){
  TimeSpan difference = rtc.now() - lastOutage;

  if (difference.totalseconds() >= WAIT_FOR_SHUTDOWN * 60){
    client.publish(SHUTDOWN_TOPIC, "Shutdown");
    DEBUG_PRINTLN("Sending Shutdown");
    shutdownFired = true;
  }
}

bool CheckIfOutagePresent(){
  float a = INA0.getCurrent_mA();
  bool status = a >= 100;

  if (status != isOutagePresent){
    DEBUG_PRINT(a);
    DEBUG_PRINT(" , ");
    DEBUG_PRINTLN(isOutagePresent);
    if (millis() - timePassedSinceOutage > 1000){
      return status;
    }
  }

  return isOutagePresent;
}