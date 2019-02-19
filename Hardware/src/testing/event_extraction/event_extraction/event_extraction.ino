#define SAMPLING_FREQ_HZ 1000
#define SAMPLING_WINDOW_SAMP 2000

void setup() {
  while(!Serial);
  delay(1000);
  Serial.begin(9600);
  Serial.println("Booting");
}

void loop() {
  Serial.println("Starting recording in...");
  delay(500);
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Recording...");
  int results[SAMPLING_WINDOW_SAMP];
  for (int i  = 0; i < SAMPLING_WINDOW_SAMP; i++) {
    results[i] = int(analogRead(A0));
    delay(1000/SAMPLING_FREQ_HZ);
  }
  Serial.println("Recording Complete");
  for (int k = 0; k < SAMPLING_WINDOW_SAMP; k++) {
    int temp = int(results[k]);
    Serial.println(temp);
  }
  while (1);
}
