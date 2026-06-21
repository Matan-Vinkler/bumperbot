#include <PID_v1.h>

// Right wheel

#define L298N_EN_A 9
#define L298N_IN_1 12
#define L298N_IN_2 13

#define RIGHT_ENCODER_PHASE_A 3
#define RIGHT_ENCODER_PHASE_B 5

// Left-wheel

#define L298N_EN_B 11
#define L298N_IN_3 7
#define L298N_IN_4 8

#define LEFT_ENCODER_PHASE_A 2
#define LEFT_ENCODER_PHASE_B 4

unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;

unsigned int left_encoder_counter = 0;
String left_encoder_sign = "p";
double left_wheel_meas_vel = 0.0;

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;

char value[] = "00.00";
uint8_t value_idx = 0;

bool is_cmd_completed = false;

bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;

double right_wheel_command_velocity = 0.0;
double left_wheel_command_velocity = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_command_velocity, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_command_velocity, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  pinMode(L298N_EN_A, OUTPUT);
  pinMode(L298N_IN_1, OUTPUT);
  pinMode(L298N_IN_2, OUTPUT);

  pinMode(L298N_EN_B, OUTPUT);
  pinMode(L298N_IN_3, OUTPUT);
  pinMode(L298N_IN_4, OUTPUT);

  pinMode(RIGHT_ENCODER_PHASE_B, INPUT);
  pinMode(LEFT_ENCODER_PHASE_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PHASE_A), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PHASE_A), leftEncoderCallback, RISING);

  digitalWrite(L298N_IN_1, HIGH);
  digitalWrite(L298N_IN_2, LOW);

  digitalWrite(L298N_IN_3, HIGH);
  digitalWrite(L298N_IN_4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
}

void loop() {
  // Reading and parsing commands from ros2_control
  if(Serial.available())
  {
    char chr = Serial.read();

    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;

      value_idx = 0;

      is_cmd_completed = false;
    }
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;

      value_idx = 0;
    }
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        digitalWrite(L298N_IN_1, HIGH - digitalRead(L298N_IN_1));
        digitalWrite(L298N_IN_2, HIGH - digitalRead(L298N_IN_2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(L298N_IN_3, HIGH - digitalRead(L298N_IN_3));
        digitalWrite(L298N_IN_4, HIGH - digitalRead(L298N_IN_4));
        is_left_wheel_forward = true;
      }
    }
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(L298N_IN_1, HIGH - digitalRead(L298N_IN_1));
        digitalWrite(L298N_IN_2, HIGH - digitalRead(L298N_IN_2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(L298N_IN_3, HIGH - digitalRead(L298N_IN_3));
        digitalWrite(L298N_IN_4, HIGH - digitalRead(L298N_IN_4));
        is_left_wheel_forward = false;
      }
    }
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_command_velocity = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_command_velocity = atof(value);
        is_cmd_completed = true;
      }

      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Calculating velocities and sending to ros2_control
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0 / 385.0)) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_command_velocity == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_command_velocity == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read = "r" + right_encoder_sign +  String(right_wheel_meas_vel) + ",l" + left_encoder_sign +  String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    last_millis = current_millis;

    analogWrite(L298N_EN_A, right_wheel_cmd);
    analogWrite(L298N_EN_B, left_wheel_cmd);
  }
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

void leftEncoderCallback()
{
  left_encoder_counter++;
  if(digitalRead(LEFT_ENCODER_PHASE_B) == HIGH)
  {
    left_encoder_sign = "n";
  }
  else
  {
    left_encoder_sign = "p";
  }
}