#include <Arduino.h>
#include "PinDefinitions.h"
#include "GlobalVariables.h"

/*********************************************************************************/
/*Function declarations*/
/*********************************************************************************/
void Kinematics(float linear_vel_cmd, float angular_vel_cmd);
void motor_write(float pwm_mtr_r, float pwm_mtr_l);
void right_wheel_pulse();
void left_wheel_pulse();
float PID_right(float &kpr, float &kdr, float &kir);
float PID_left(float &kpl, float &kdl, float &kil);
void report();
void interface();

/*********************************************************************************/
/*Main loop*/
/*********************************************************************************/
void setup()
{
  attachInterrupt(digitalPinToInterrupt(R_CHA), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(L_CHA), left_wheel_pulse, RISING);

  pinMode(R_CHA, INPUT_PULLUP);
  pinMode(R_CHB, INPUT);

  pinMode(L_CHA, INPUT_PULLUP);
  pinMode(L_CHB, INPUT);

  pinMode(R_MOTORA, OUTPUT);
  pinMode(R_MOTORB, OUTPUT);

  pinMode(L_MOTORA, OUTPUT);
  pinMode(L_MOTORB, OUTPUT);

  Serial.begin(57600);
}

void loop()
{
  currentMillis = millis();

  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;

    rpm_right = (float)(right_wheel_pulse_count * 60 / (R_ENCount * interval * 0.001));
    rpm_left = (float)(left_wheel_pulse_count * 60 / (L_ENCount * interval * 0.001));

    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
  }

  pwm_right = PID_right(Kpr, Kdr, Kir);
  pwm_left = PID_left(Kpl, Kdl, Kil);

  motor_write(pwm_right, pwm_left);
}

void Kinematics(float linear_vel_cmd, float angular_vel_cmd)
{
  // Calculate the left and right wheel velocities using the kinematics of a differential drive robot
  left_wheel_vel = ((2 * linear_vel_cmd) - (angular_vel_cmd * WHEEL_DISTANCE)) / (2 * WHEEL_RADIUS);
  right_wheel_vel = ((2 * linear_vel_cmd) + (angular_vel_cmd * WHEEL_DISTANCE)) / (2 * WHEEL_RADIUS);

  // Convert the left and right wheel velocities to RPMs
  ref_l = left_wheel_vel * 60 / (2 * PI);
  ref_r = right_wheel_vel * 60 / (2 * PI);

  ref_l = constrain(ref_l, -MAX_RPM, MAX_RPM);
  ref_r = constrain(ref_r, -MAX_RPM, MAX_RPM);
}

void motor_write(float pwm_mtr_r, float pwm_mtr_l)
{
  if (pwm_mtr_r >= 0)
  {
    analogWrite(R_MOTORB, pwm_mtr_r);
    analogWrite(R_MOTORA, 0);
  }
  else
  {
    analogWrite(R_MOTORA, abs(pwm_mtr_r));
    analogWrite(R_MOTORB, 0);
  }

  if (pwm_mtr_l >= 0)
  {
    analogWrite(L_MOTORB, pwm_mtr_l);
    analogWrite(L_MOTORA, 0);
  }
  else
  {
    analogWrite(L_MOTORA, abs(pwm_mtr_l));
    analogWrite(L_MOTORB, 0);
  }
}

void right_wheel_pulse()
{
  int val = digitalRead(R_CHB);

  if (val == LOW)
  {
    Direction_right = true; // Reverse
  }
  else
  {
    Direction_right = false; // Forward
  }

  if (Direction_right)
  {
    right_wheel_pulse_count++;
  }
  else
  {
    right_wheel_pulse_count--;
  }
}

void left_wheel_pulse()
{
  int val = digitalRead(L_CHB);

  if (val == LOW)
  {
    Direction_left = false; // Reverse
  }
  else
  {
    Direction_left = true; // Forward
  }

  if (Direction_left)
  {
    left_wheel_pulse_count++;
  }
  else
  {
    left_wheel_pulse_count--;
  }
}

float PID_right(float &kpr, float &kdr, float &kir)
{
  error_r = ref_r - rpm_right;
  pwm_right = (Kpr * error_r) + kdr * (error_r - prev_error_r) + kir * sum_error_r;
  prev_error_r = error_r;
  sum_error_r = sum_error_r + error_r;
  pwm_right = constrain(pwm_right, -255, 255);
  return pwm_right;
}

float PID_left(float &kpl, float &kdl, float &kil)
{
  error_l = ref_l - rpm_left;
  pwm_left = (kpl * error_l) + kdl * (error_l - prev_error_l) + kil * sum_error_l;
  prev_error_l = error_l;
  sum_error_l = sum_error_l + error_l;
  pwm_left = constrain(pwm_left, -255, 255);
  return pwm_left;
}

void report()
{
  Serial.print("ReferenceRight:");
  Serial.print(ref_r);
  Serial.print(" RightSpeed:");
  Serial.print(rpm_right);

  Serial.print(" Referenceleft:");
  Serial.print(ref_l);
  Serial.print(" LeftSpeed:");
  Serial.println(rpm_left);
}

void interface()
{
  if (Serial.available() > 0)
  {
    Serial.readBytes((char *)&Kpr, sizeof(float));
    Serial.readBytes((char *)&Kir, sizeof(float));
    Serial.readBytes((char *)&Kdr, sizeof(float));
    Serial.readBytes((char *)&ref_r, sizeof(float));

    Serial.readBytes((char *)&Kpl, sizeof(float));
    Serial.readBytes((char *)&Kil, sizeof(float));
    Serial.readBytes((char *)&Kdl, sizeof(float));
    Serial.readBytes((char *)&ref_l, sizeof(float));
  }
}