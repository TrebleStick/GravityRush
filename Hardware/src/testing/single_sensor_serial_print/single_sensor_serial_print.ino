void setup() {
#define sensor_1_in A0
while(!Serial);
pinMode(sensor_1_in, INPUT);
Serial.begin(9600);
Serial.println("Starting");
delay(1000);
}

void loop() {
  int reading = analogRead(A0);
  Serial.println(reading);
  delay(20);
}
