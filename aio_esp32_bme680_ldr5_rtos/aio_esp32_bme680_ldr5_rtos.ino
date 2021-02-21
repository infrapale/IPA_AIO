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
uint16_t   ldr_value[NBR_LDR_RES];
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



static SemaphoreHandle_t  aio_sema_handle;
void StartTasks(void){
void TaskReadBme680( void *pvParameters );
void TaskReadLight( void *pvParameters );
void TaskSendAio( void *pvParameters );
void TaskPrintValues( void *pvParameters );

void select_ldr(uint8_t ldr_idx);


// 

void setup() {
    BaseType_t rc;
    // start the serial connection
    Serial.begin(115200);

    // wait for serial monitor to open
    while(! Serial);

    pinMode(LED_YELLOW, OUTPUT);
    digitalWrite(LED_YELLOW,LOW);

    pinMode(LDR_PIN,INPUT);
    select_ldr(0);

    Serial.print("Connecting to Adafruit IO");
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
    StartTasks();

}

void loop() {
    /* Let tasks do the job */   
}


void StartTasks(void){
    void TaskReadBme680( void *pvParameters );
    void TaskReadLight( void *pvParameters );
    void TaskSendAio( void *pvParameters );
    void TaskPrintValues( void *pvParameters );

    xTaskCreatePinnedToCore(
       TaskReadBme680
        ,  "TaskReadBme680"   // A name just for humans
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
       TaskReadLight
        ,  "TaskReadLight"   // A name just for humans
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
       TaskSendAio
        ,  "TaskSendAio"   // A name just for humans
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
       TaskPrintValues
        ,  "TaskPrintValues"   // A name just for humans
        ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL 
        ,  ARDUINO_RUNNING_CORE);

    aio_sema_handle = xSemaphoreCreateBinary();
    assert(aio_sema_handle);
    rc = xTaskCreatePinnedToCore(
        range_task,
        "rangetsk",
        2048, // Stack size
        nullptr,
        1, // Priority
        &h, // Task handle
        app_cpu // CPU
    );
    assert(rc == pdPASS);
    assert(h);


}



void TaskReadBme680( void *pvParameters ){
    unsigned long endTime;


    if (!bme.begin(0x76)) {
        while (1){
            Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
            vTaskDelay(10000);
        }
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    //bme.setGasHeater(320, 150); // 320*C for 150 ms
    bme.setGasHeater(0, 0); // 320*C for 150 ms

    for (;;)
    {    
        endTime = bme.beginReading();
        if (endTime == 0) {
            Serial.println(F("Failed to begin reading :("));
            return;
        } 
        else {
            vTaskDelay(1000);     
            if (!bme.endReading()) {
                Serial.println(F("Failed to complete reading :("));
                // return;
            } else {
                // values are now stored in the BME680 object
            }
        }
        vTaskDelay(1000);     
    }  
}

void TaskReadLight( void *pvParameters ){
    uint8_t ldr_indx = 0;
    
    for (;;)
    {    
        ldr_value[ldr_indx] = analogRead(LDR_PIN);
        if (++ldr_indx) >= NBR_LDR_RES ) {
            ldr_indx = 0;
        }
        select_ldr(ldr_indx);
        vTaskDelay(1000);
    }
}

void TaskSendAio( void *pvParameters ){
    uint8_t iot_state = 0;
    
    for (;;)
    {    
        io.run()
        switch (iot_state) {
            case 0: case 1: case 2: case 3: case 4:
                if (iot_state < NBR_LDR_RES){
                    ldr_feed[iot_state]->save(ldr_value[iot_state]);
                }    
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
        vTaskDelay(5000);
    }
}



void TaskPrintValues(void){
    //BaseType_t rc;
    for (;;) {

        Serial.print(F("Temp:     ")); Serial.print(bme.temperature); Serial.println(F(" C"));
        Serial.print(F("Hum:      ")); Serial.print(bme.humidity); Serial.println(F(" %"));
        Serial.print(F("LDR:      ")); Serial.print(ldr_value); Serial.println(F(" "));
        for (uint_8 i = 0; i < NBR_LDR_RES; i++){
            Serial.print(F("LDR:  ")); 
            Serial.print(i); 
            Serial.print(":  "); 
            Serial.print(ldr_value[i]); 
            Serial.println(F(" "));  
        }
        vTaskDelay(10000);
    }     
        //rc = xSemaphoreTake(aio_sema_handle,0);
        //if (rc == pdPASS){   
}

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


void handleMessage(AdafruitIO_Data *data) {

  Serial.print("received <- ");
  Serial.print(data->toPinLevel());
  if(data->toPinLevel() == 1)
    Serial.println("HIGH");
  else
    Serial.println("LOW");

  //digitalWrite(LED_RED, data->toPinLevel());
}
