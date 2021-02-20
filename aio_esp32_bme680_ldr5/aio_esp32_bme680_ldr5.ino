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
//#include "secrets.h"
#include "config.h"


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <bme680.h>
#include <bme680_defs.h>

//AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
/*
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define BTN_A    36
#define BTN_B    39
#define LED_RED      26
#define LED_GREEN    32
*/
#define LED_YELLOW   33
/*
#define LED_BLUE     25
#define LDR_PIN      34
*/
#define LDR_PIN      34
#define NBR_LDR_RES  5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
byte            iot_state = 0;
/************************ Example Starts Here *******************************/

// this int will hold the current count for our sketch
int count = 0;
uint16_t   ldr_value;
uint8_t ldr_select_pin[NBR_LDR_RES] = {15,16,17,18,19};

// set up the 'counter' feed
AdafruitIO_Feed *temperature = io.feed("villaastrid.tupa-bme680-temp");
AdafruitIO_Feed *humidity    = io.feed("villaastrid.tupa-bme680-humidity");
// AdafruitIO_Feed *led_red     = io.feed("home-tampere.esp32test-led-red");
AdafruitIO_Feed *ldr_feed[] = {
  io.feed("villaastrid.tupa-ldr-1"),
  io.feed("villaastrid.tupa-ldr-2"),
  io.feed("villaastrid.tupa-ldr-3"),
  io.feed("villaastrid.tupa-ldr-4"),
  io.feed("villaastrid.tupa-ldr-5") 
};

void select_ldr(uint8_t ldr_idx){
    for (uint8_t i =0;i<5;i++)
    {
        if(i==ldr_idx) {
           pinMode(ldr_select_pin[i], OUTPUT); 
           digitalWrite(ldr_select_pin[i], LOW);
        } 
        else {
           pinMode(ldr_select_pin[i], INPUT);        
        }
      
    }
   pinMode(LED_YELLOW, OUTPUT);
}

// 
//infrapale/feeds/home-tampere.esp32test-temp"
void setup() {

    // start the serial connection
    Serial.begin(115200);

    // wait for serial monitor to open
    while(! Serial);

    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LDR_PIN,INPUT);
    digitalWrite(LED_YELLOW,HIGH);

    select_ldr(99);  /* none selected */
 
    Serial.print("Connecting to Adafruit IO");
    Serial.println(F("BME680 test"));
    delay(2000);
    if (!bme.begin(0x76)) {
        Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
        while (1);
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    //bme.setGasHeater(320, 150); // 320*C for 150 ms
    bme.setGasHeater(0, 0); // 320*C for 150 ms


    
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
   
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
        Serial.println(F("Failed to begin reading :("));
        return;
    }
 
    Serial.print(F("Reading started at "));
    Serial.print(millis());
    Serial.print(F(" and will finish at "));
    Serial.println(endTime);

    Serial.println(F("You can do other work during BME680 measurement."));
    delay(50); // This represents parallel work.
    // There's no need to delay() until millis() >= endTime: bme.endReading()
    // takes care of that. It's okay for parallel work to take longer than
    // BME680's measurement time.

    // Obtain measurement results from BME680. Note that this operation isn't
    // instantaneous even if milli() >= endTime due to I2C/SPI latency.

    if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        // return;
    }
    // Send and receive data from AIO
    switch (iot_state) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            select_ldr(iot_state);
            delay(1);
            ldr_value = analogRead(LDR_PIN);
            Serial.print(F("LDR:  ")); Serial.print(iot_state); Serial.print(":  "); Serial.print(ldr_value); Serial.println(F(" "));  
            ldr_feed[iot_state]->save(ldr_value);
            break;

        case 5:
            temperature->save(bme.temperature);
            break;
        case 6:    
            humidity->save(bme.humidity);
            break;
    }
    iot_state++;
    if (iot_state > 6 ){
      iot_state = 0;
    } 
    //led_red->onMessage(handleMessage);
    
    Serial.print(F("Reading completed at "));
    Serial.println(millis());

    Serial.print(F("Temp:     ")); Serial.print(bme.temperature); Serial.println(F(" C"));
    Serial.print(F("Hum:      ")); Serial.print(bme.humidity); Serial.println(F(" %"));
    Serial.print(F("LDR:      ")); Serial.print(ldr_value); Serial.println(F(" "));
 
  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events. In this example, we will wait three seconds
  // (1000 milliseconds == 1 second) during each loop.
  delay(3000);

}

void handleMessage(AdafruitIO_Data *data) {

  Serial.print("received <- ");
  Serial.print(data->toPinLevel());
  if(data->toPinLevel() == 1)
    Serial.println("HIGH");
  else
    Serial.println("LOW");

  //digitalWrite(LED_RED, data->toPinLevel());
}
