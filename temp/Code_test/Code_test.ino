#include "config.h"
#include "control_motor.h"
#include "Read_RX.h"
#include "MPU_control.h"

// Timing variables
unsigned long lastMPURead = 0;
const unsigned long MPU_INTERVAL = 10;  // 10ms = 100Hz

void setup() {
  setup_RX();        // Khởi tạo UART (Serial 9600 baud)
  setup_motor();     // Khởi tạo motor và encoder
  setup_MPU();       // Khởi tạo MPU9250 với Madgwick filter
  
  Serial.println(F("System ready!"));
}

void loop() {
  // 1. Đọc MPU (100Hz)
  unsigned long now = millis();
  if (now - lastMPURead >= MPU_INTERVAL) {
    updateMPU();  // Cập nhật giá trị yaw
    lastMPURead = now;
  }
  
  // 2. Đọc dữ liệu từ UART
  read_RX();
  
  // 3. Điều khiển motor với/không hiệu chỉnh yaw
  if (is_data_timeout()) {
    control(0, 0, 0, 0);  // Dừng cả 2 motor để an toàn
    keepStraight = false; // Tắt chế độ giữ hướng
  } else {
    controlWithYawCorrection();  // Điều khiển có hiệu chỉnh yaw
  }
}

// Hàm điều khiển motor với hiệu chỉnh yaw khi đi thẳng/lùi
void controlWithYawCorrection() {
  int L_rpm_adj = L_rpm;
  int R_rpm_adj = R_rpm;
  
  // Kiểm tra xem có đang đi thẳng hoặc lùi không
  // Điều kiện: cùng chiều (dir giống nhau) và có tốc độ
  bool isGoingStraight = (L_dir == R_dir) && (L_dir != 0) && (L_rpm > 0) && (R_rpm > 0);
  
  if (isGoingStraight) {
    // Lần đầu tiên đi thẳng → lưu yaw mục tiêu
    if (!keepStraight) {
      resetTargetYaw();
      Serial.print(F("Target Yaw set: "));
      Serial.println(targetYaw);
    }
    
    // Lấy giá trị hiệu chỉnh từ MPU
    float correction = getYawCorrection();
    
    // Hiệu chỉnh RPM:
    // - Lệch PHẢI (correction < 0): giảm trái, tăng phải
    // - Lệch TRÁI (correction > 0): tăng trái, giảm phải
    L_rpm_adj = constrain(L_rpm + correction, 0, 500);
    R_rpm_adj = constrain(R_rpm - correction, 0, 500);
    
    // Debug (optional - có thể comment để giảm overhead)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print(F("Yaw: "));
      Serial.print(currentYaw, 1);
      Serial.print(F(" Target: "));
      Serial.print(targetYaw, 1);
      Serial.print(F(" Correction: "));
      Serial.println(correction, 1);
      lastDebug = millis();
    }
  } else {
    // Không đi thẳng → tắt chế độ giữ hướng
    keepStraight = false;
  }
  
  // Gọi PID với RPM đã điều chỉnh
  control(L_dir, L_rpm_adj, R_dir, R_rpm_adj);
}