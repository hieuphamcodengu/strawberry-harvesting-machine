#ifndef READ_RX_H
#define READ_RX_H

#include <Arduino.h>

// Biến lưu trữ dữ liệu nhận được
int L_dir = 0;         // Hướng động cơ trái (0=dừng, 1=thuận, 2=nghịch)
int L_rpm = 0;         // RPM động cơ bên trái
int R_dir = 0;         // Hướng động cơ phải (0=dừng, 1=thuận, 2=nghịch)
int R_rpm = 0;         // RPM động cơ bên phải

String inputString = "";     // Chuỗi nhận được
bool stringComplete = false; // Đánh dấu nhận đủ chuỗi

// Biến timeout để bảo vệ an toàn
unsigned long lastDataTime = 0;  // Thời gian nhận dữ liệu cuối cùng
const unsigned long TIMEOUT_MS = 1000;  // Timeout 1 giây

// Hàm khởi tạo UART
void setup_RX() {
  Serial.begin(9600); // UART phần cứng (chân 0-RX, 1-TX)
  inputString.reserve(30); // Dự trữ bộ nhớ cho chuỗi
  
  // Khởi tạo lastDataTime để tránh timeout ngay khi khởi động
  lastDataTime = millis();
}

// Hàm đọc dữ liệu từ UART
// Format: "dir_L,rpm_L,dir_R,rpm_R#"
// Ví dụ: "1,100,2,50#" = Trái thuận 100rpm, Phải nghịch 50rpm
void read_RX() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '#') {
      stringComplete = true;
      break; // Thoát ngay khi nhận đủ chuỗi
    } else if (inChar == '\n' || inChar == '\r') {
      // Bỏ qua ký tự xuống dòng
      continue;
    } else {
      inputString += inChar;
      
      // Bảo vệ tràn buffer
      if (inputString.length() > 25) {
        inputString = "";
      }
    }
  }
  
  if (stringComplete) {
    // Phân tích chuỗi dạng: dir_L,rpm_L,dir_R,rpm_R#
    int firstComma = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);
    int thirdComma = inputString.indexOf(',', secondComma + 1);
    
    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
      L_dir = inputString.substring(0, firstComma).toInt();
      L_rpm = inputString.substring(firstComma + 1, secondComma).toInt();
      R_dir = inputString.substring(secondComma + 1, thirdComma).toInt();
      R_rpm = inputString.substring(thirdComma + 1).toInt();
      
      // Giới hạn giá trị hợp lệ
      L_dir = constrain(L_dir, 0, 2);
      R_dir = constrain(R_dir, 0, 2);
      L_rpm = constrain(L_rpm, 0, 500);
      R_rpm = constrain(R_rpm, 0, 500);
      
      // Cập nhật thời gian nhận dữ liệu
      lastDataTime = millis();
    }
    
    // Xóa buffer và cờ
    inputString = "";
    stringComplete = false;
  }
}

// Hàm kiểm tra timeout - trả về true nếu quá lâu không nhận dữ liệu
bool is_data_timeout() {
  return (millis() - lastDataTime) > TIMEOUT_MS;
}

#endif // READ_RX_H
