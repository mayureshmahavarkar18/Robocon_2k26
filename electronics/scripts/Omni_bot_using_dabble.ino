/*
 * IMPORTANT: This code requires ESP32 board package version 2.0.17 or lower
 * 
 * To downgrade:
 * 1. Go to Tools → Board → Boards Manager
 * 2. Search "esp32"
 * 3. Select version 2.0.17
 * 4. Click Install
 * 5. Restart Arduino IDE
 * 
 * Then install DabbleESP32 library from Library Manager
 */

#include <DabbleESP32.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include "Adafruit_BNO08x.h"

// BNO
Adafruit_BNO08x bno08x(-1);  // -1 = no reset pin used
float yaw = 0;

// ESP32 default light pin
const int light = 2;

// motor pin1 or pin
const int motor1_pin = 16;
const int motor2_pin = 5;
const int motor3_pin1 = 13;

// motor pin2 or dir 18, 5
const int motor1_dir = 17;
const int motor2_dir = 18;
const int motor3_pin2 = 12;

// Encoder pins
ESP32Encoder motor1_enc;
ESP32Encoder motor2_enc;
ESP32Encoder motor3_enc;

// Encoder variables for speed calculation
double motor1_enc_old_position = 0;
double motor2_enc_old_position = 0;
double motor3_enc_old_position = 0;
unsigned long motor1_enc_last_time = 0;
unsigned long motor2_enc_last_time = 0;
unsigned long motor3_enc_last_time = 0;

// important variables
int w1 = 0, w2 = 0, w3 = 0;  // angular velocity of wheels
int maxspeed = 127;          // maxspeed while controlling bot by buttons

// Physical Constants
const float WHEEL_RADIUS = 1.0f;
const float WHEEL_TO_CENTRE_DISTANCE = 1.0f;
const int TICKS_PER_REVOLUTION = 1300;
const int MAX_MOTOR_RPM_AT_FULL_PWM = 450;

// PID Gains
const double Kp = 0.8, Ki = 0, Kd = 0.003004;
const double Kp_bno = 6, Ki_bno = 0, Kd_bno = 0;

// Calculated Constants
const double RPM_TO_PWM_SCALE = 255.0 / MAX_MOTOR_RPM_AT_FULL_PWM;
const double MAX_RPM_MOTOR_REQUIRED = 350.0;
const double JOYSTICK_TO_RPM_K = MAX_RPM_MOTOR_REQUIRED / 127.0;

// PID Controller class
class PIDController {
public:
  double Kp, Ki, Kd;
  double integral, prev_error;
  unsigned long last_time;
  double integral_max, integral_min;

  PIDController(double p, double i, double d)
    : Kp(p), Ki(i), Kd(d),
      integral(0), prev_error(0), last_time(0) {
    if (Ki != 0) {
      integral_max = 255.0 / Ki;
      integral_min = -255.0 / Ki;
    } else {
      integral_max = 0;
      integral_min = 0;
    }
  }

  void reset() {
    integral = 0;
    prev_error = 0;
    last_time = micros();
  }

  double compute(double setpoint, double actual) {
    unsigned long now = micros();
    double dt = (double)(now - last_time) / 1000000.0;
    last_time = now;

    if (dt == 0) return (Kp * prev_error) + (Ki * integral) + (Kd * 0);

    double error = setpoint - actual;
    integral += error * dt;

    if (integral > integral_max) {
      integral = integral_max;
    } else if (integral < integral_min) {
      integral = integral_min;
    }

    double derivative = (error - prev_error) / dt;
    prev_error = error;

    return (Kp * error) + (Ki * integral) + (Kd * derivative);
  }
};

// PID & CONTROL VARIABLES
PIDController pid1(Kp, Ki, Kd);
PIDController pid2(Kp, Ki, Kd);
PIDController pid3(Kp, Ki, Kd);
PIDController pid_bno(Kp_bno, Ki_bno, Kd_bno);

void setup() {
  Serial.begin(115200);
  
  // Dabble setup - Change "OmniBot" to your preferred Bluetooth name
  Dabble.begin("OmniBot");
  Serial.println("Dabble Started. Connect via Dabble App.");
  
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  motor1_enc.attachFullQuad(27, 26);
  motor2_enc.attachFullQuad(35, 34);
  motor3_enc.attachFullQuad(25, 33);
  
  Serial.println("Encoders Ready.");
  
  // BNO
  Wire.begin(21, 22);  // SDA=21, SCL=22
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO085 over I2C!");
    while (1) {
      digitalWrite(light, HIGH);
      delay(100);
      digitalWrite(light, LOW);
      delay(100);
    }
  }
  Serial.println("BNO085 found over I2C");

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable rotation vector");
  }

  pinMode(light, OUTPUT);
  pinMode(motor1_pin, OUTPUT);
  pinMode(motor2_pin, OUTPUT);
  pinMode(motor3_pin1, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor3_pin2, OUTPUT);
  
  Serial.println("System Ready!");
}

void calculate_kinematis(int x, int y, int w, int &w1, int &w2, int &w3) {
  const float V1_BY_3 = 0.33333333f;
  const float V2_BY_3 = 0.66666667f;
  const float V1_SQRT_3 = 0.57735027f;
  
  float fw1 = (V2_BY_3 * y + V1_BY_3 * w * WHEEL_TO_CENTRE_DISTANCE) / WHEEL_RADIUS * JOYSTICK_TO_RPM_K;
  float fw2 = (-V1_SQRT_3 * x - V1_BY_3 * y + V1_BY_3 * w * WHEEL_TO_CENTRE_DISTANCE) / WHEEL_RADIUS * JOYSTICK_TO_RPM_K;
  float fw3 = (V1_SQRT_3 * x - V1_BY_3 * y + V1_BY_3 * w * WHEEL_TO_CENTRE_DISTANCE) / WHEEL_RADIUS * JOYSTICK_TO_RPM_K;

  w1 = (int)fw1;
  w2 = (int)fw2;
  w3 = (int)fw3;
}

double theta_setpoint = 0;
bool start_setpoint = true;
bool exit_code = false;

void loop() {
  Dabble.processInput();  // Refresh Dabble data
  
  if (exit_code) {
    setMotor1(motor1_pin, motor1_dir, 0);
    setMotor1(motor2_pin, motor2_dir, 0);
    setMotor2(motor3_pin1, motor3_pin2, 0);
    digitalWrite(light, LOW);
    return;
  }
  
  // BNO sensor reading
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;

      float ysqr = qy * qy;
      float t3 = +2.0f * (qw * qz + qx * qy);
      float t4 = +1.0f - 2.0f * (ysqr + qz * qz);
      yaw = atan2(t3, t4) * 57.45;
      
      if (start_setpoint) {
        theta_setpoint = yaw;
        start_setpoint = false;
      }
    }
  }
  
  // Check if Dabble is connected
  if (Dabble.isAppConnected()) {
    digitalWrite(light, HIGH);
    
    // Exit on Cross button
    if (GamePad.isCrossPressed()) {
      exit_code = true;
      return;
    }

    // Rotation control using Square and Circle buttons
    int w = 0;
    if (GamePad.isCirclePressed()) {  // Rotate right
      w = 127;
    } else if (GamePad.isSquarePressed()) {  // Rotate left
      w = -127;
    }

    int x = 0, y = 0;
    
    // Controlling bot by arrow buttons
    if (GamePad.isUpPressed() || GamePad.isDownPressed() || 
        GamePad.isLeftPressed() || GamePad.isRightPressed()) {
      int a = 0, b = 0;
      if (GamePad.isUpPressed()) a = 1;
      else if (GamePad.isDownPressed()) a = -1;
      if (GamePad.isLeftPressed()) b = 1;
      else if (GamePad.isRightPressed()) b = -1;
      x = maxspeed * a;
      y = maxspeed * b;
    } else {
      // Joystick control
      int angle = GamePad.getAngle();
      int radius = GamePad.getRadius();
      
      if (radius > 0) {
        // Convert polar to cartesian (-127 to 127)
        float rad = angle * PI / 180.0;
        x = (int)((radius / 7.0) * 127.0 * cos(rad));
        y = (int)((radius / 7.0) * 127.0 * sin(rad));
      }
    }
    
    // Reset orientation on Triangle button
    if (GamePad.isTrianglePressed()) {
      theta_setpoint = yaw;
    }

    // Apply deadzone
    if (abs(x) < 10) x = 0;
    if (abs(y) < 10) y = 0;
    if (abs(w) < 10) w = 0;
    
    if (w) {
      theta_setpoint = yaw;
      calculate_kinematis(x, y, w, w1, w2, w3);
    } else {
      if (yaw != theta_setpoint) {
        double w_actual;
        if (theta_setpoint - yaw > 180) 
          w_actual = (int)(pid_bno.compute(theta_setpoint - 360, yaw) * 0.7);
        else if (theta_setpoint - yaw < -180) 
          w_actual = (int)(pid_bno.compute(theta_setpoint + 360, yaw) * 0.7);
        else 
          w_actual = (int)(pid_bno.compute(theta_setpoint, yaw) * 0.7);
        calculate_kinematis(x, y, w_actual, w1, w2, w3);
      } else {
        pid_bno.reset();
        calculate_kinematis(x, y, 0, w1, w2, w3);
      }
    }

    if (w1 == 0 && w2 == 0 && w3 == 0) {
      setMotor1(motor1_pin, motor1_dir, 0);
      setMotor1(motor2_pin, motor2_dir, 0);
      setMotor2(motor3_pin1, motor3_pin2, 0);
      pid1.reset();
      pid2.reset();
      pid3.reset();
    } else {
      double w1_actual = get_actual_rpm(motor1_enc, motor1_enc_old_position, motor1_enc_last_time);
      double pwm1 = pid1.compute(w1, w1_actual) * RPM_TO_PWM_SCALE;
      setMotor1(motor1_pin, motor1_dir, pwm1);

      double w2_actual = get_actual_rpm(motor2_enc, motor2_enc_old_position, motor2_enc_last_time);
      double pwm2 = pid2.compute(w2, w2_actual) * RPM_TO_PWM_SCALE;
      setMotor1(motor2_pin, motor2_dir, pwm2);

      double w3_actual = get_actual_rpm(motor3_enc, motor3_enc_old_position, motor3_enc_last_time);
      double pwm3 = pid3.compute(w3, w3_actual) * RPM_TO_PWM_SCALE;
      setMotor2(motor3_pin1, motor3_pin2, pwm3);
    }
  } else {
    // Blink LED when disconnected
    digitalWrite(light, (millis() / 1000) % 2);
  }
}

void setMotor1(int motor_pin, int motor_dir, double pwm) {
  int output_pwm = constrain((int)abs(pwm), 0, 255);
  if (pwm >= 0) {
    digitalWrite(motor_dir, HIGH);
  } else {
    digitalWrite(motor_dir, LOW);
  }
  analogWrite(motor_pin, output_pwm);
}

void setMotor2(int motor_pin1, int motor_pin2, double pwm) {
  int output_pwm = constrain((int)abs(pwm), 0, 255);
  if (pwm >= 0) {
    analogWrite(motor_pin1, output_pwm);
    analogWrite(motor_pin2, 0);
  } else {
    analogWrite(motor_pin1, 0);
    analogWrite(motor_pin2, output_pwm);
  }
}

double get_actual_rpm(ESP32Encoder &enc, double &old_position, unsigned long &last_time) {
  unsigned long now = micros();
  int current_position = enc.getCount();

  double dt = (double)(now - last_time) / 1000000.0;

  if (last_time == 0 || dt == 0) {
    old_position = current_position;
    last_time = now;
    return 0.0;
  }

  double ticks_per_second = (current_position - old_position) / dt;
  double rpm = (ticks_per_second / TICKS_PER_REVOLUTION);

  old_position = current_position;
  last_time = now;
  return rpm;
}
