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

//#include "secrets.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <bme680.h>
#include <bme680_defs.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define BTN_A    36
#define BTN_B    39
#define LED_RED      26
#define LED_GREEN    32
#define LED_YELLOW   33
#define LED_BLUE     25
#define LDR_PIN      34

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);


Adafruit_BME680 bme; // I2C

/************************ Example Starts Here *******************************/

// this int will hold the current count for our sketch
int count = 0;
uint16_t   ldr_value;

// set up the 'counter' feed
AdafruitIO_Feed *temperature = io.feed("home-tampere.esp32test-temp");
AdafruitIO_Feed *humidity    = io.feed("home-tampere.esp32test-hum");
AdafruitIO_Feed *led_red     = io.feed("home-tampere.esp32test-led-red");
AdafruitIO_Feed *ldr_feed    = io.feed("home-tampere.esp32test-ldr");


// 
//infrapale/feeds/home-tampere.esp32test-temp"
void setup() {

    // start the serial connection
    Serial.begin(115200);

    // wait for serial monitor to open
    while(! Serial);

    pinMode(BTN_A, INPUT_PULLUP);
    pinMode(BTN_B, INPUT_PULLUP);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_GREEN,HIGH);
    
 
    Serial.print("Connecting to Adafruit IO");
    Serial.println(F("BME680 test"));
    delay(2000);
    if (!bme.begin(0x77)) {
        Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
        while (1);
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    //bme.setGasHeater(320, 150); // 320*C for 150 ms
    bme.setGasHeater(0, 0); // 320*C for 150 ms


    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();

    // Draw a single pixel in white
    display.drawPixel(10, 10, WHITE);

    // Show the display buffer on the screen. You MUST call display() after
    // drawing commands to make them visible on screen!
    display.display();
    delay(2000);

    display.clearDisplay();

    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("Hello, world!"));

    
    // connect to io.adafruit.com
    io.connect();

    // wait for a connection
    while(io.status() < AIO_CONNECTED) {
        switch (io.status()) 
        {
            case AIO_IDLE: Serial.println(F("Waiting for connection establishement")); break;  
            case AIO_NET_DISCONNECTED: Serial.println(F("Network disconnected")); break;
            case AIO_DISCONNECTED: Serial.println(F("Disconnected from Adafruit IO")); break;
            case AIO_FINGERPRINT_UNKOWN: Serial.println(F("Unknown AIO_SSL_FINGERPRINT")); break;
            case AIO_NET_CONNECT_FAILED: Serial.println(F("Failed to connect to network")); break;
            case AIO_CONNECT_FAILED: Serial.println(F("Failed to connect to Adafruit IO")); break; 
            case AIO_FINGERPRINT_INVALID: Serial.println(F("Unknown AIO_SSL_FINGERPRINT")); break; 
            case AIO_AUTH_FAILED: Serial.println(F("Invalid Adafruit IO login credentials provided")); break; 
            case AIO_SSID_INVALID: Serial.println(F("SSID is "" or otherwise invalid, connection not attempted")); break;

            case AIO_NET_CONNECTED: Serial.println(F("Connected to Adafruit IO")); break; 
            case AIO_CONNECTED: Serial.println(F(" Connected to network")); break; 
            case AIO_CONNECTED_INSECURE: Serial.println(F("Insecurely (non-SSL) connected to network")); break; 
            case AIO_FINGERPRINT_UNSUPPORTED: Serial.println(F("Unsupported AIO_SSL_FINGERPRINT")); break; 
            case AIO_FINGERPRINT_VALID: Serial.println(F("Valid AIO_SSL_FINGERPRINT")); break;    
            default: Serial.println(F("Unkown AIO status")); break;
        }
       
        delay(5000);
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
    static uint8_t feed_indx = 0;
    
    if (endTime == 0) {
        Serial.println(F("Failed to begin reading :("));
        return;
    }
 
    Serial.print(F("Reading started at "));
    Serial.print(millis());
    Serial.print(F(" and will finish at "));
    Serial.println(endTime);

    delay(50); // This represents parallel work.
    // There's no need to delay() until millis() >= endTime: bme.endReading()
    // takes care of that. It's okay for parallel work to take longer than
    // BME680's measurement time.

    // Obtain measurement results from BME680. Note that this operation isn't
    // instantaneous even if milli() >= endTime due to I2C/SPI latency.

    if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        return;
    }
    // Send and receive data from AIO
    switch(feed_indx)
    {
        case 0:
            temperature->save(bme.temperature);
            break;
        case 1:
            humidity->save(bme.humidity);
            break;
        case 2:
            ldr_value = analogRead(LDR_PIN);
            ldr_feed->save(ldr_value);
            //led_red->onMessage(handleMessage);
            break;
    }
    feed_indx++;
    if (feed_indx > 2) feed_indx = 0;
    
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
 
 
   delay(1*60*1000);

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
