#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include "BluefruitConfig.h"

/* The following libraries need to be installed: Adafruit BluefruitLE nRF51; Adafruit LIS3DH; Adafruit Unified Sensor */
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
//#include <Adafruit_LIS3DH.h>
//#include <Adafruit_Sensor.h>

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define EMG_Pin_1                     A0
#define EMG_Pin_2                     A2
//#define SDA                         A4
//#define SCL                         A5
#define SAMPLING_WINDOW_HZ           1000
#define SAMPLING_WINDOW_SIZE         500

// If windowing is required uncomment this #define. If not then streaming will be used.
//#define WINDOWING

//int sensorPin = A0;
float val = 0;
int setting = 0;

// Setup software serial
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BLUEFRUIT_UART_MOD_PIN, OUTPUT);

  Serial.begin(115200);   // setup hardware serial
  Serial.println(F("GravityRush Firmware"));
  Serial.println(F("------------------------------------------------"));

  // Set module to CMD mode
  Serial.println( F("Switching to CMD mode!") );
  //ble.setMode(BLUEFRUIT_MODE_COMMAND);
  digitalWrite(BLUEFRUIT_UART_MOD_PIN, HIGH);

  Serial.println(F("******************************"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'GravityRush BLE': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=GravityRush BLE" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID"));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable HID"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  Serial.println(F("Please connect to the module via your phone's Bluetooth settings"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  //ble.setMode(BLUEFRUIT_MODE_DATA);
  digitalWrite(BLUEFRUIT_UART_MOD_PIN, LOW);

  Serial.println(F("******************************"));

  /* Set up accelerometer */
  /*Serial.println("LIS3DH test!");

    if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
    }
    Serial.println("LIS3DH found!");

    lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

    Serial.print("LIS Range = "); Serial.print(2 << lis.getRange());
    Serial.println("G");*/
}

void loop() {
  // Read EMG Data
  // GET RYAN TO SCALE VALUES
  val_1 = analogRead(EMG_Pin_1);
  val_2 = analogRead(EMG_Pin_2);
  //Serial.println(val);           // print sensor value

  // Read Raw accel data
  //lis.read();

  // Send EMG data via bluetooth in stream mode
  if (val != 0) {
    // GET RYAN TO DO UNICODE STUFF WITH THE ,
    ble.print(val_1);
    ble.print(",");
    ble.print(val_2);
  }

  //send EMG
//#ifdef WINDOWING
//  float data[SAMPLING_WINDOW_SIZE];
//  for (int i = 0; i < SAMPLING_WINDOW_SIZE; i++) {
//    data[i] = analogRead(EMG_Pin);
//    delay(1000 / SAMPLING_WINDOW_HZ);
//  }
//
//  for (int i = 0; i < SAMPLING_WINDOW_SIZE; i++) {
//    data[i] = data[i] / 1023;
//    ble.print(data[i]);
//  }
//#endif

  // Send raw accel data via bluetooth
  /*if ((lis.x != 0) || (lis.y != 0) || (lis.z != 0)) {
    ble.print("X:  "); ble.print(lis.x);
    ble.print("  \tY:  "); ble.print(lis.y);
    ble.print("  \tZ:  "); ble.print(lis.z);
    }*/

  // Receive data via bluetooth
  if (ble.available()) {
    setting = ble.read();
    Serial.print((char)setting);
  }

  delay(100);
}

// Chain: Read Sensor (AnalogRead) (and accelerometer) -> Send via Bluetooth
// Additional: Receive instruction from Bluetooth -> Change sensor parameters?
