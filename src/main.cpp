#include "./config.h"

#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include "EspMQTTClient.h"
#include <WiFiUdp.h>
#include <NTPClient.h>


#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif


bool isOutagePresent = false;
bool timeSetted = false;
bool shutdownFired = false;
unsigned long int timePassedSinceOutage = 0;
unsigned long int timePassed = 0;

static int taskCore = 0;

RTC_DS3231 rtc;
DateTime lastOutage;
WiFiUDP wifiUdp;

NTPClient timeClient(wifiUdp, "at.pool.ntp.org", TIMEZONE_DELTA * 3600, 60000);

EspMQTTClient client(
  WIFI_SSID,
  WIFI_PASSWORD,
  MQTT_BROKER_IP,  // MQTT Broker server ip
  MQTT_USERNAME,   // Can be omitted if not needed
  MQTT_PASSWORD,   // Can be omitted if not needed
  MQTT_CLIENT_NAME,     // Client name that uniquely identify your device
  MQTT_PORT              // The MQTT port, default to 1883. this line can be omitted
);


ICACHE_RAM_ATTR void onOutagePinChange();
ICACHE_RAM_ATTR void onPowerRestored();
void coreTask( void * pvParameters );
void lostpower();

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
  #endif

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("RTC not detected");

    while (1); // Hang indefinitely if RTC is not found
  }

  pinMode(OUTAGE_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OUTAGE_SENSOR_PIN), onOutagePinChange, CHANGE);

  xTaskCreatePinnedToCore(
    coreTask,   /* Function to implement the task */
    "coreTask", /* Name of the task */
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    0,          /* Priority of the task */
    NULL,       /* Task handle. */
    taskCore);  /* Core where the task should run */
}

void loop() {
  if (isOutagePresent && !timeSetted){
    timePassedSinceOutage = millis();
    timeSetted = true;
    lastOutage = rtc.now();
    DEBUG_PRINTLN("Outage detected");
    client.publish(OUTAGE_TOPIC, "toSend");
  }
  else if(!isOutagePresent && timeSetted){
    timeSetted = false;
    shutdownFired = false;
    DEBUG_PRINTLN("Power restored");
    client.publish(POWER_RESTORED_TOPIC, "toSend");
  }

  if (isOutagePresent && !shutdownFired){
    TimeSpan difference = rtc.now() - lastOutage;

    if (difference.totalseconds() >= WAIT_FOR_SHUTDOWN * 60){
      client.publish(SHUTDOWN_TOPIC, "Shutdown");
      DEBUG_PRINTLN("Sending Shutdown");
      shutdownFired = true;
    }
  }

  client.loop();
  delay(10);
}

ICACHE_RAM_ATTR void onOutagePinChange() {
  if (digitalRead(OUTAGE_SENSOR_PIN) == LOW)
    isOutagePresent = true;
  else
    isOutagePresent = false;
}

void onConnectionEstablished() {
  timeClient.begin();
  timeClient.update();
  timeClient.forceUpdate();

  if (rtc.lostPower()){
    lostpower();
  }
}

void coreTask( void * pvParameters ){
    while(true){
      if (millis() - timePassed >= DELAY_BETWEEN_STATUS * 1000){
        String toSend;
        DynamicJsonDocument doc(2048);

        DateTime now = rtc.now();

        doc["lastOutageTime"] = lastOutage.timestamp() + "Z";
        doc["isOutagePresent"] = isOutagePresent;
        doc["time"] = now.timestamp() + "Z";
        doc["voltage"] = 12;
        doc["current"] = 2.302038;
        doc["power"] = 24;

        serializeJson(doc, toSend);

        client.publish(STATUS_TOPIC, toSend);
        timePassed = millis();
        DEBUG_PRINTLN("status message sent");
      }

      vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void lostpower() {
  timeClient.update();

  rtc.adjust(DateTime(timeClient.getEpochTime()));
}