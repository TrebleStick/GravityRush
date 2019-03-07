#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>

#define EMG_PIN                      A0
#define STIM_LED                     8

int SAMPLING_WINDOW_HZ = 1000; 
int SAMPLING_WINDOW_SIZE = 500;

// If windowing is required uncomment this #define. If not then streaming will be used.
//#define WINDOWING

//int sensorPin = A0;
float val = 0;
int setting = 0;
bool shake = false; 

bool comms = true; 

void setup() {
//  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STIM_LED, OUTPUT);
  pinMode(EMG_PIN, INPUT); 
   
  Serial.begin(115200);
  Serial.println("Setting up Serial");    
//  Serial.println("GravityRush Firmware");
//  Serial.println("------------------------------------------------");

  Serial.println("Beginning handshake"); 
  while (!shake) {
    shake = handShake();
    delay(5); 
  } 
  
  Serial.println(shake);
  Serial.println("End of shake");
  clearBuffer();       
}
  
void loop() {

  // Stop serial communication
  if (Serial.available()) {
    if (Serial.read() == 'e') {
      Serial.println("Ending comms"); 
      digitalWrite(STIM_LED, LOW); 
      clearBuffer();       
      Serial.end(); 
      comms = false; 
    }
  }

  if (comms) {
    //send EMG

    // float data[SAMPLING_WINDOW_SIZE];
    int data[SAMPLING_WINDOW_SIZE];

    // float data = analogRead(EMG_PIN)/1023;
    // Serial.println(data); 
     
    digitalWrite(STIM_LED, HIGH); 
    delay(250);
    digitalWrite(STIM_LED, LOW);
    delay(250);
    digitalWrite(STIM_LED, HIGH); 
    delay(250);
    digitalWrite(STIM_LED, LOW); 
    delay(250);
    digitalWrite(STIM_LED, HIGH);
    delay(125); 
    // delay(1000);
    // digitalWrite(STIM_LED, LOW); 
    // delay(1000);
    // digitalWrite(STIM_LED, HIGH);

    // delay(1000); 
    for(int i = 0; i < SAMPLING_WINDOW_SIZE; i++) {
  //    data[i] = analogRead(EMG_Pin);
      
      data[i] = analogRead(EMG_PIN);
      //data[i] = i; 
      delay(1000/SAMPLING_WINDOW_HZ);
    }  
  
    digitalWrite(STIM_LED, LOW);
    // delay(5000); */ 
    
    for(int i = 0; i < SAMPLING_WINDOW_SIZE; i++) { 
      if (i < (SAMPLING_WINDOW_SIZE - 1)) {
        // String value = 'v' + ',' + (String)data[i]; 
        String value = "v,"; 
        value += (String)data[i];
        // value += (String)i; 
        Serial.println(value); 
        delay(10);
      } else {
        // Serial.println('s' + ',' + (String)data[i]); 
        String value = "s,"; 
        value += (String)data[i];
        // value += (String)i; 
        Serial.println(value); 
        delay(10); 
      }
    }   
  }
  
  delay(100);
}

bool handShake() { 
  if (Serial.available()) { 
    if (Serial.read() == 'f') {
      Serial.println("Return true"); 
      delay(5); 
      return true; 
    } 
  } else { 
    Serial.println("b");
    delay(100); 
  }  
  return false; 
}

void clearBuffer() {
  char bin; 
  while (Serial.read() == 'f') {
     bin = Serial.read(); 
  }
}

// Chain: Read Sensor (AnalogRead) (and accelerometer) -> Send via Bluetooth
// Additional: Receive instruction from Bluetooth -> Change sensor parameters?
