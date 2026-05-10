#define L298N_EN_A 9
#define L298N_IN_1 12
#define L298N_IN_2 13

#define L298N_EN_B 11
#define L298N_IN_3 7
#define L298N_IN_4 8

double command_1 = 0.0;
double command_2 = 0.0;

void setup() {
  pinMode(L298N_EN_A, OUTPUT);
  pinMode(L298N_IN_1, OUTPUT);
  pinMode(L298N_IN_2, OUTPUT);

  pinMode(L298N_EN_B, OUTPUT);
  pinMode(L298N_IN_3, OUTPUT);
  pinMode(L298N_IN_4, OUTPUT);

  digitalWrite(L298N_IN_1, HIGH);
  digitalWrite(L298N_IN_2, LOW);

  digitalWrite(L298N_IN_3, HIGH);
  digitalWrite(L298N_IN_4, LOW);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int separator = data.indexOf(',');
    if (separator != -1) {
      command_1 = data.substring(0, separator).toDouble();
      command_2 = data.substring(separator + 1).toDouble();
    } else {
      command_1 = data.toDouble();
      command_2 = command_1;
    }
  }

  int pwm_1 = (int)(fabs(command_1) * 255);
  int pwm_2 = (int)(fabs(command_2) * 255);

  analogWrite(L298N_EN_A, pwm_1);
  analogWrite(L298N_EN_B, pwm_2);
}
