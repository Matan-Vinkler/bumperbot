#define L298N_EN_A 9
#define L298N_IN_1 12
#define L298N_IN_2 13

double command = 0.0;

void setup() {
  pinMode(L298N_EN_A, OUTPUT);
  pinMode(L298N_IN_1, OUTPUT);
  pinMode(L298N_IN_2, OUTPUT);

  digitalWrite(L298N_IN_1, HIGH);
  digitalWrite(L298N_IN_2, LOW);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    command = Serial.readString().toDouble();
  }

  int pwm = (int)(fabs(command) * 255);
  analogWrite(L298N_EN_A, pwm);
}
