#include "config.h"
#include <util/atomic.h>

// Forward declarations
void M1_pwm(int pwmVal);
void M2_pwm(int pwmVal);
void readEncoder_M1();
void readEncoder_M2();
void M1(float target);
void M2(float target);

// globals M1
long prevT_M1 = 0;
int posPrev_M1 = 0;
float e_prev_M1 = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i_M1 = 0;
volatile float velocity_i_M1 = 0;
volatile long prevT_i_M1 = 0;

float v1Filt_M1 = 0;
float v1Prev_M1 = 0;
float v2Filt_M1 = 0;
float v2Prev_M1 = 0;

float eintegral_M1 = 0;
// globals M2
long prevT_M2 = 0;
int posPrev_M2 = 0;
float e_prev_M2 = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i_M2 = 0;
volatile float velocity_i_M2 = 0;
volatile long prevT_i_M2 = 0;

float v1Filt_M2 = 0;
float v1Prev_M2 = 0;
float v2Filt_M2 = 0;
float v2Prev_M2 = 0;

float eintegral_M2 = 0;

// Thông số PID và Ramping (có thể điều chỉnh)
float kp = 2;        // Hệ số tỷ lệ
float ki = 1.7;        // Hệ số tích phân
float kd = 0.1;       // Hệ số đạo hàm
float rampRate = 30.0; // RPM/giây - tốc độ tăng target

void setup_motor() {
  // Cấu hình chân điều khiển động cơ
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_1_EN1, OUTPUT);
  pinMode(MOTOR_1_EN2, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  pinMode(MOTOR_2_EN1, OUTPUT);
  pinMode(MOTOR_2_EN2, OUTPUT);
  
  // Cấu hình encoder
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  
  // Gắn interrupt cho encoder
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder_M1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder_M2, RISING);
}


void M1(float target) {
  // Soft-start: tăng target dần dần để tránh vọt
  static float smoothTarget_M1 = 0;
  float rampRate = 40.0; // RPM/giây - tăng để bắt target nhanh hơn
  
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i_M1;
    velocity2 = velocity_i_M1;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float)(currT - prevT_M1)) / 1.0e6;
  float velocity1 = (pos - posPrev_M1) / deltaT;
  posPrev_M1 = pos;
  prevT_M1 = currT;
  
  // Ramping: tăng dần target
  float maxChange = rampRate * deltaT;
  if (target > smoothTarget_M1) {
    smoothTarget_M1 += min(maxChange, target - smoothTarget_M1);
  } else if (target < smoothTarget_M1) {
    smoothTarget_M1 -= min(maxChange, smoothTarget_M1 - target);
  }
  target = smoothTarget_M1; // Sử dụng target đã làm mượt

  // Convert count/s to RPM
  float v1 = velocity1 / 250 * 60.0;
  float v2 = velocity2 / 250 * 60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt_M1 = 0.854 * v1Filt_M1 + 0.0728 * v1 + 0.0728 * v1Prev_M1;
  v1Prev_M1 = v1;
  v2Filt_M1 = 0.854 * v2Filt_M1 + 0.0728 * v2 + 0.0728 * v2Prev_M1;
  v2Prev_M1 = v2;

  // Compute the control signal u (sử dụng biến toàn cục)
  float e = target - v1Filt_M1;                   // Sai số hiện tại
  
  // Anti-windup: chỉ tích lũy khi sai số trong ngưỡng hợp lý
  if (fabs(e) < 100) {
    eintegral_M1 = eintegral_M1 + e * deltaT;
  }
  
  // Giới hạn integral để tránh windup
  eintegral_M1 = constrain(eintegral_M1, -100, 100);
  
  float e_derivative = (e - e_prev_M1) / deltaT;  // Đạo hàm sai số

  // Tính toán tín hiệu điều khiển
  float u = kp * e + ki * eintegral_M1 + kd * e_derivative;
  e_prev_M1 = e;  // Cập nhật e_prev cho lần tính toán sau

  // Set the motor speed and direction
  int pwr = (int)fabs(u);
  if (pwr > 254) {
    pwr = 254;
  }
  M1_pwm(pwr);

  Serial.print(target);
  Serial.print(" ");
  Serial.print(v1Filt_M1);
}
void M2(float target) {
  // Soft-start: tăng target dần dần để tránh vọt
  static float smoothTarget_M2 = 0;
  float rampRate = 40.0; // RPM/giây - tăng để bắt target nhanh hơn
  
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i_M2;
    velocity2 = velocity_i_M2;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float)(currT - prevT_M2)) / 1.0e6;
  float velocity1 = (pos - posPrev_M2) / deltaT;
  posPrev_M2 = pos;
  prevT_M2 = currT;
  
  // Ramping: tăng dần target
  float maxChange = rampRate * deltaT;
  if (target > smoothTarget_M2) {
    smoothTarget_M2 += min(maxChange, target - smoothTarget_M2);
  } else if (target < smoothTarget_M2) {
    smoothTarget_M2 -= min(maxChange, smoothTarget_M2 - target);
  }
  target = smoothTarget_M2; // Sử dụng target đã làm mượt

  // Convert count/s to RPM
  float v1 = velocity1 / 250 * 60.0;
  float v2 = velocity2 / 250 * 60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt_M2 = 0.854 * v1Filt_M2 + 0.0728 * v1 + 0.0728 * v1Prev_M2;
  v1Prev_M2 = v1;
  v2Filt_M2 = 0.854 * v2Filt_M2 + 0.0728 * v2 + 0.0728 * v2Prev_M2;
  v2Prev_M2 = v2;

  // Compute the control signal u (sử dụng biến toàn cục)
  float e = target - v1Filt_M2;                   // Sai số hiện tại
  
  // Anti-windup: chỉ tích lũy khi sai số trong ngưỡng hợp lý
  if (fabs(e) < 100) {
    eintegral_M2 = eintegral_M2 + e * deltaT;
  }
  
  // Giới hạn integral để tránh windup
  eintegral_M2 = constrain(eintegral_M2, -100, 100);
  
  float e_derivative = (e - e_prev_M2) / deltaT;  // Đạo hàm sai số

  // Tính toán tín hiệu điều khiển
  float u = kp * e + ki * eintegral_M2 + kd * e_derivative;
  e_prev_M2 = e;  // Cập nhật e_prev cho lần tính toán sau

  // Set the motor speed and direction
  int pwr = (int)fabs(u);
  if (pwr > 254) {
    pwr = 254;
  }
  M2_pwm(pwr);
  Serial.print(" ");
  Serial.println(v1Filt_M2);
}
// Function to stop the motor
void stopMotor() {
  M1_pwm(0);
  M2_pwm(0);
  digitalWrite(MOTOR_1_EN1, LOW);
  digitalWrite(MOTOR_1_EN2, LOW);
  digitalWrite(MOTOR_2_EN1, LOW);
  digitalWrite(MOTOR_2_EN2, LOW);
}

// Function to set motor parameters
void M1_pwm(int pwmVal) {
  analogWrite(MOTOR_1_PWM, pwmVal);  // Motor speed
}
void M2_pwm(int pwmVal) {
  analogWrite(MOTOR_2_PWM, pwmVal);  // Motor speed
}
void readEncoder_M1() {
  //Read encoder A
  int increment = 1;
  pos_i_M1 = pos_i_M1 + increment;
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i_M1)) / 1.0e6;
  velocity_i_M1 = increment / deltaT;
  prevT_i_M1 = currT;
}
void readEncoder_M2() {
  // Read encoder B
  int increment = 1;
  pos_i_M2 = pos_i_M2 + increment;
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i_M2)) / 1.0e6;
  velocity_i_M2 = increment / deltaT;
  prevT_i_M2 = currT;
}

void control(int dir_L, int target_L, int dir_R, int target_R) {
  // Biến static để lưu target trước đó
  static int prev_target_L = 0;
  static int prev_target_R = 0;
  static int prev_dir_L = 0;
  static int prev_dir_R = 0;
  
  // Control left motor
  if (dir_L == 1) {
    // Nếu thay đổi chiều hoặc target thay đổi lớn, reset PID
    if (prev_dir_L != dir_L || abs(target_L - prev_target_L) > 50) {
      eintegral_M1 = 0;
      e_prev_M1 = 0;
    }
    digitalWrite(MOTOR_1_EN1, LOW);
    digitalWrite(MOTOR_1_EN2, HIGH);
    M1(target_L);
    prev_target_L = target_L;
    prev_dir_L = dir_L;
  } else if (dir_L == 2) {
    // Nếu thay đổi chiều hoặc target thay đổi lớn, reset PID
    if (prev_dir_L != dir_L || abs(target_L - prev_target_L) > 50) {
      eintegral_M1 = 0;
      e_prev_M1 = 0;
    }
    digitalWrite(MOTOR_1_EN1, HIGH);
    digitalWrite(MOTOR_1_EN2, LOW);
    M1(target_L);
    prev_target_L = target_L;
    prev_dir_L = dir_L;
  } else {
    digitalWrite(MOTOR_1_EN1, LOW);
    digitalWrite(MOTOR_1_EN2, LOW);
    M1_pwm(0);
    // Reset PID
    eintegral_M1 = 0;
    e_prev_M1 = 0;
    prev_target_L = 0;
    prev_dir_L = 0;
  }

  // Control right motor
  if (dir_R == 1) {
    // Nếu thay đổi chiều hoặc target thay đổi lớn, reset PID
    if (prev_dir_R != dir_R || abs(target_R - prev_target_R) > 50) {
      eintegral_M2 = 0;
      e_prev_M2 = 0;
    }
    digitalWrite(MOTOR_2_EN1, LOW);
    digitalWrite(MOTOR_2_EN2, HIGH);
    M2(target_R);
    prev_target_R = target_R;
    prev_dir_R = dir_R;
  } else if (dir_R == 2) {
    // Nếu thay đổi chiều hoặc target thay đổi lớn, reset PID
    if (prev_dir_R != dir_R || abs(target_R - prev_target_R) > 50) {
      eintegral_M2 = 0;
      e_prev_M2 = 0;
    }
    digitalWrite(MOTOR_2_EN1, HIGH);
    digitalWrite(MOTOR_2_EN2, LOW);
    M2(target_R);
    prev_target_R = target_R;
    prev_dir_R = dir_R;
  } else {
    digitalWrite(MOTOR_2_EN1, LOW);
    digitalWrite(MOTOR_2_EN2, LOW);
    M2_pwm(0);
    // Reset PID
    eintegral_M2 = 0;
    e_prev_M2 = 0;
    prev_target_R = 0;
    prev_dir_R = 0;
  }
}