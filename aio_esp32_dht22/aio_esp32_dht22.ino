// Adafruit IO Publish Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
//#define  VILLA_ASTRID 1
#define  LILLA_ASTRID 1
#include "config.h"

#include "DHTesp.h"
#include "Ticker.h"

#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

#include "secrets.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>


/************************ Example Starts Here *******************************/

// this int will hold the current count for our sketch
int count = 0;
uint16_t   ldr_value;
DHTesp dht;

void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;
/** Pin number for DHT11 data pin */
int dhtPin = 5;


// set up the 'counter' feed
AdafruitIO_Feed *temperature = io.feed("home-tampere.esp32test-temp");
AdafruitIO_Feed *humidity    = io.feed("home-tampere.esp32test-hum");
AdafruitIO_Feed *led_red     = io.feed("home-tampere.esp32test-led-red");
AdafruitIO_Feed *ldr_feed    = io.feed("home-tampere.esp32test-ldr");


/**
 * initTemp
 * Setup DHT library
 * Setup task and timer for repeated measurement
 * @return bool
 *    true if task and timer are started
 *    false if task or timer couldn't be started
 */
bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      tempTask,                       /* Function to implement the task */
      "tempTask ",                    /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &tempTaskHandle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    tempTicker.attach(20, triggerGetTemp);
  }
  return true;
}


// 
//infrapale/feeds/home-tampere.esp32test-temp"
void setup() {

    // start the serial connection
    Serial.begin(115200);

    // wait for serial monitor to open
    while(! Serial);

    pinMode(BTN_A, INPUT_PULLUP);
    digitalWrite(LED_GREEN,HIGH);
    
 
    Serial.print("Connecting to Adafruit IO");
    delay(2000); // Pause for 2 seconds
    // connect to io.adafruit.com
    io.connect();

    // wait for a connection
    while(io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    // we are connected
    Serial.println();
    Serial.println(io.statusText());

}

void loop() {

    // io.run(); is required for all sketches.
    // it should always be present at the top of your loop
    // function. it keeps the client connected to
    // io.adafruit.com, and processes any incoming data.
    io.run();
  
    // save count to the 'counter' feed on Adafruit IO
   
    // Send and receive data from AIO

    temperature->save(bme.temperature);
    humidity->save(bme.humidity);
    ldr_value = analogRead(LDR_PIN);
    ldr_feed->save(ldr_value);
    led_red->onMessage(handleMessage);
    
    Serial.print(F("Reading completed at "));
    Serial.println(millis());
    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner

    display.print(F("Temp:     ")); display.print(bme.temperature); display.println(F(" C"));
    display.print(F("Hum:      ")); display.print(bme.humidity); display.println(F(" %"));
    display.print(F("LDR:      ")); display.print(ldr_value); display.println(F(" "));
    display.display();
 
  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events. In this example, we will wait three seconds
  // (1000 milliseconds == 1 second) during each loop.
  delay(60000);

}

void handleMessage(AdafruitIO_Data *data) {

  Serial.print("received <- ");
  Serial.print(data->toPinLevel());
  if(data->toPinLevel() == 1)
    Serial.println("HIGH");
  else
    Serial.println("LOW");

  digitalWrite(LED_RED, data->toPinLevel());
}
