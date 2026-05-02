#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  if(Serial.available())
  {
    int x = Serial.readString().toInt();
    digitalWrite(LED_PIN, x == 0 ? LOW : HIGH);
  }
}
