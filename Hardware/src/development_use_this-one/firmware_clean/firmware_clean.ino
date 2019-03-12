#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include "BluefruitConfig.h"

/* The following libraries need to be installed: Adafruit BluefruitLE nRF51; Adafruit LIS3DH; Adafruit Unified Sensor */
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define EMG_Pin_1                     A0
#define EMG_Pin_2                     A2
#define SAMPLING_WINDOW_HZ           1000
#define SAMPLING_WINDOW_SIZE         500

uint16_t val_1 = 0;
uint16_t val_2 = 0;
int setting = 0;

// Setup software serial
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// Error printing function
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BLUEFRUIT_UART_MODE_PIN, OUTPUT);

  Serial.begin(115200);   // setup hardware serial
  Serial.println(F("GravityRush Firmware"));
  Serial.println(F("------------------------------------------------"));

  // Set module to CMD mode
  Serial.println( F("Switching to CMD mode!") );
  //ble.setMode(BLUEFRUIT_MODE_COMMAND);
  digitalWrite(BLUEFRUIT_UART_MODE_PIN, HIGH);

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
  digitalWrite(BLUEFRUIT_UART_MODE_PIN, LOW);

  Serial.println(F("******************************"));
}

void loop() {
  // Read EMG Data - Ryan needs to deal with scaling
  val_1 = analogRead(EMG_Pin_1);
  val_2 = analogRead(EMG_Pin_2);

  // Send EMG data via bluetooth in stream mode - Ryan needs to deal with Unicode for stuff including the comma
  if (val_1 != 0 && val_2 != 0) {
    String data = (String)val_1 + "," + (String)val_2 + "\n";
    Serial.print(data);
    ble.print(data);
  }
  //If both values are 0 then send -1,-1 over BLE
  else {
    val_1 = -1;
    val_2 = -1;
    String data = (String)val_1 + "," + (String)val_2 + "\n";
    Serial.print(data);
    ble.print(data);
  }

  // Receive data via bluetooth
  if (ble.available()) {
    setting = ble.read();
    Serial.print((char)setting);
  }
  delay(1000 / SAMPLING_WINDOW_HZ);
}
