#include <Wire.h>

/* ================= MPU ================= */
#define MPU_ADDR 0x68

#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define CONFIG 0x1A  // DLPF
#define ACCEL_XOUT_H 0x3B

/* ================= Madgwick ================= */
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float beta = 0.08f;

// Timing
unsigned long lastUpdate = 0;

// Gyro bias
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

/* ================= LPF (software) ================= */
float gx_f = 0, gy_f = 0, gz_f = 0;
float ax_f = 0, ay_f = 0, az_f = 0;

const float alpha = 0.2f;  // 0.1–0.3 (0.2 khuyến nghị)

/* ================= I2C ================= */
void writeByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readAccelGyro(int16_t *data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  for (int i = 0; i < 7; i++) {
    data[i] = Wire.read() << 8 | Wire.read();
  }
}

/* ================= GYRO CALIB ================= */
void calibrateGyro() {
  const int samples = 600;
  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t raw[7];

  Serial.println("=== CALIB GYRO ===");
  Serial.println("Giu IMU DUNG YEN...");
  delay(1000);

  for (int i = 0; i < samples; i++) {
    readAccelGyro(raw);
    sumX += raw[4];
    sumY += raw[5];
    sumZ += raw[6];
    delay(5);
  }

  gyroBiasX = sumX / (float)samples;
  gyroBiasY = sumY / (float)samples;
  gyroBiasZ = sumZ / (float)samples;

  Serial.println("Calib xong.");
}

/* ================= Madgwick Update ================= */
void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float dt) {

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  if (ax != 0 || ay != 0 || az != 0) {
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    s0 = 4 * q0 * q2q2 + 2 * q2 * ax + 4 * q0 * q1q1 - 2 * q1 * ay;
    s1 = 4 * q1 * q3q3 - 2 * q3 * ax + 4 * q0q0 * q1 - 2 * q0 * ay;
    s2 = 4 * q0q0 * q2 + 2 * q0 * ax + 4 * q2 * q3q3 - 2 * q3 * ay;
    s3 = 4 * q1q1 * q3 - 2 * q1 * ax + 4 * q2q2 * q3 - 2 * q2 * ay;

    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;
  } else {
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("=== MPU9250 CLONE YAW (6DOF + FILTER) ===");

  writeByte(PWR_MGMT_1, 0x00);
  delay(100);

  writeByte(CONFIG, 0x03);        // DLPF ~42Hz
  writeByte(GYRO_CONFIG, 0x00);   // ±250 dps
  writeByte(ACCEL_CONFIG, 0x00);  // ±2g

  calibrateGyro();
  lastUpdate = micros();
}

/* ================= LOOP ================= */
void loop() {
  int16_t raw[7];
  readAccelGyro(raw);

  float ax = raw[0] / 16384.0f;
  float ay = raw[1] / 16384.0f;
  float az = raw[2] / 16384.0f;

  float gx = (raw[4] - gyroBiasX) / 131.0f * DEG_TO_RAD;
  float gy = (raw[5] - gyroBiasY) / 131.0f * DEG_TO_RAD;
  float gz = (raw[6] - gyroBiasZ) / 131.0f * DEG_TO_RAD;

  // ===== SOFTWARE LPF =====
  gx_f = alpha * gx + (1 - alpha) * gx_f;
  gy_f = alpha * gy + (1 - alpha) * gy_f;
  gz_f = alpha * gz + (1 - alpha) * gz_f;

  ax_f = alpha * ax + (1 - alpha) * ax_f;
  ay_f = alpha * ay + (1 - alpha) * ay_f;
  az_f = alpha * az + (1 - alpha) * az_f;

  unsigned long now = micros();
  float dt = (now - lastUpdate) * 1e-6f;
  lastUpdate = now;

  madgwickUpdate(gx_f, gy_f, gz_f,
                 ax_f, ay_f, az_f,
                 dt);

  float yaw = atan2(2 * (q1 * q2 + q0 * q3),
                    q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
              * RAD_TO_DEG;

  Serial.print("YAW (deg): ");
  Serial.println(yaw);

  delay(10);
}
