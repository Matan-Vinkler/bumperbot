#define L298N_EN_A 9
#define L298N_IN_1 12
#define L298N_IN_2 13

#define RIGHT_ENCODER_PHASE_A 3
#define RIGHT_ENCODER_PHASE_B 5

unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;

void setup() {
  pinMode(L298N_EN_A, OUTPUT);
  pinMode(L298N_IN_1, OUTPUT);
  pinMode(L298N_IN_2, OUTPUT);
  pinMode(RIGHT_ENCODER_PHASE_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PHASE_A), rightEncoderCallback, RISING);

  digitalWrite(L298N_IN_1, HIGH);
  digitalWrite(L298N_IN_2, LOW);

  Serial.begin(115200);
}

void loop() {
  right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
  String encoder_read = "r" + right_encoder_sign +  String(right_wheel_meas_vel);
  Serial.println(encoder_read);

  analogWrite(L298N_EN_A, 255);

  right_encoder_counter = 0;
  delay(100);
}

void rightEncoderCallback()
{
  right_encoder_counter++;
  if(digitalRead(RIGHT_ENCODER_PHASE_B) == HIGH)
  {
    right_encoder_sign = "p";
  }
  else
  {
    right_encoder_sign = "n";
  }
}