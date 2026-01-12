#include <AccelStepper.h>
#include <MultiStepper.h>
#include <SoftwareSerial.h>

// SoftwareSerial để nhận tín hiệu từ ESP32
// A1 = RX (nhận), A0 = TX (truyền - không dùng)
SoftwareSerial mySerial(A1, A0); // RX, TX

// Định nghĩa các chân kết nối - Motor 1 (TMC2208 - Arduino 5V)
#define STEP_PIN_1 5
#define DIR_PIN_1 6
#define EN_PIN_1 7

// Định nghĩa các chân kết nối - Motor 2 (MD5-HD14 - Arduino 5V)
#define STEP_PIN_2 3
#define DIR_PIN_2 4
#define EN_PIN_2 2

// Công tắc homing (endstop/limit switch)
#define HOME_SWITCH_Y 8  // Công tắc home cho trục Y (Motor 1)
#define HOME_SWITCH_Z 9  // Công tắc home cho trục Z (Motor 2)

// ===== CONFIG MOTOR 1 (GT2 Pulley) =====
// NEMA 17: 200 steps/vòng (1.8 độ/step)
// TMC2208 Microstepping:
// - MS1=LOW, MS2=LOW: 1/8 step   -> 1600 steps/vòng
// - MS1=HIGH, MS2=LOW: 1/2 step  -> 400 steps/vòng
// - MS1=LOW, MS2=HIGH: 1/4 step  -> 800 steps/vòng
// - MS1=HIGH, MS2=HIGH: 1/16 step -> 3200 steps/vòng
// - Không nối (hoặc NC): 1/8 step (mặc định) -> 1600 steps/vòng

#define MICROSTEP_1 8                                         // Microstep = 8 (1/8 step)
#define STEPS_PER_REV_1 200                                   // NEMA 17: 200 steps/vòng
#define STEPS_PER_ROTATION_1 (STEPS_PER_REV_1 * MICROSTEP_1)  // = 1600 steps/vòng

// GT2 Pulley và Linear Motion cho Motor 1
// GT2 Belt: Pitch = 2mm
// GT2 Pulley 20 răng: Chu vi = 20 * 2mm = 40mm/vòng
// Steps per mm = 1600 steps/vòng / 40mm/vòng = 40 steps/mm
#define GT2_PULLEY_TEETH_1 20                                      // Số răng pulley GT2
#define GT2_BELT_PITCH_1 2.0                                       // Khoảng cách răng GT2 (mm)
#define MM_PER_ROTATION_1 (GT2_PULLEY_TEETH_1 * GT2_BELT_PITCH_1)  // = 40mm/vòng
#define STEPS_PER_MM_1 (STEPS_PER_ROTATION_1 / MM_PER_ROTATION_1)  // = 40 steps/mm

// ===== CONFIG MOTOR 2 (Motor có hộp giảm tốc + Vít me T8) =====
// NEMA 17 + Hộp giảm tốc: 0.072°/step -> 5000 steps/vòng (360°/0.072° = 5000)
// MD5-HD14 Driver: Cấu hình microstepping
#define MICROSTEP_2 1                                         // Microstep = 1 (full step)
#define STEPS_PER_REV_2 5000                                  // 5000 steps/vòng (bao gồm hộp giảm tốc)
#define STEPS_PER_ROTATION_2 (STEPS_PER_REV_2 * MICROSTEP_2)  // = 5000 steps/vòng

// Vít me T8 (Lead Screw T8)
// T8 = đường kính 8mm, bước 2mm (lead/pitch = 2mm)
// 1 vòng motor = 2mm di chuyển tuyến tính
// => T2 2 sẽ gửi 5000 steps = 1 vòng = 2mm
#define LEADSCREW_PITCH_2 8.0                                      // Bước vít me (mm/vòng)
#define MM_PER_ROTATION_2 LEADSCREW_PITCH_2                        // = 2mm/vòng
#define STEPS_PER_MM_2 (STEPS_PER_ROTATION_2 / MM_PER_ROTATION_2)  // = 2500 steps/mm

// Tạo đối tượng AccelStepper với driver type 1 (DRIVER)
// AccelStepper::DRIVER: chỉ cần STEP và DIR
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// Tạo đối tượng MultiStepper để điều khiển nhiều motor
MultiStepper steppers;

// Biến điều khiển cho từng motor
bool isMoving1 = false;   // Motor 1 đang chạy
bool isMoving2 = false;   // Motor 2 đang chạy
bool needSendDone = false; // Cần gửi DONE# sau khi hoàn thành cả 2 motor
bool isPending1 = false;  // Motor 1 đang chờ Motor 2 hoàn thành
bool isContinuous1 = false; // Motor 1 quay liên tục
bool isContinuous2 = false; // Motor 2 quay liên tục
char inputBuffer[32];     // Buffer lưu lệnh từ Serial (char array thay vì String)
uint8_t bufferIndex = 0;  // Vị trí hiện tại trong buffer

// Biến nhận dữ liệu từ ESP32: z,y# (tọa độ tuyệt đối)
float targetZ = 0;   // Tọa độ Z mục tiêu (mm) - Motor 2
float targetY = 0;   // Tọa độ Y mục tiêu (mm) - Motor 1
float currentZ = 0;  // Tọa độ Z hiện tại (mm) - Motor 2
float currentY = 0;  // Tọa độ Y hiện tại (mm) - Motor 1
bool isHomed = false; // Đã thực hiện homing chưa
bool isHoming = false; // Đang trong quá trình homing

// Tham số vận tốc cho từng motor (đơn vị mm)
float currentSpeed1 = 300.0;  // Tốc độ Motor 1 (mm/s)
float currentAccel1 = 150;    // Gia tốc Motor 1 (mm/s^2)
float currentSpeed2 = 10.0;   // Tốc độ Motor 2 (mm/s)
float currentAccel2 = 3;   // Gia tốc Motor 2 (mm/s^2)

// ===== GIỚI HẠN HÀNH TRÌNH (mm) =====
// Thay đổi các giá trị này theo hành trình thực tế của máy
const float MAX_Z = 200.0;  // Hành trình tối đa trục Z (Motor 2) - mm
const float MAX_Y = 300.0;  // Hành trình tối đa trục Y (Motor 1) - mm

// ===== KHOẢNG CÁCH MỖI LẦN DI CHUYỂN REALTIME (mm) =====
const float REALTIME_STEP_Z = 5.0;  // Mỗi lần move trục Z khi nhấn giữ nút (mm)
const float REALTIME_STEP_Y = 10.0; // Mỗi lần move trục Y khi nhấn giữ nút (mm)

void setup() {
  Serial.begin(115200);  // Serial cho debug
  mySerial.begin(9600);  // SoftwareSerial nhận từ ESP32
  delay(1000);

  Serial.println(F("=== Khoi tao 2 Stepper Motor ==="));
  Serial.println();

  Serial.println(F("[MOTOR 1] NEMA 17 + GT2 Pulley 20 rang"));
  Serial.print(F("  Microstep: 1/"));
  Serial.println(MICROSTEP_1);
  Serial.print(F("  Steps/vong: "));
  Serial.println(STEPS_PER_ROTATION_1);
  Serial.print(F("  Chu vi pulley: "));
  Serial.print(MM_PER_ROTATION_1);
  Serial.println(F(" mm/vong"));
  Serial.print(F("  Steps/mm: "));
  Serial.println(STEPS_PER_MM_1);
  Serial.println();

  Serial.println(F("[MOTOR 2] NEMA 17 - Truyen dong"));
  Serial.print(F("  Microstep: 1/"));
  Serial.println(MICROSTEP_2);
  Serial.print(F("  Steps/vong: "));
  Serial.println(STEPS_PER_ROTATION_2);
  Serial.print(F("  Ty le chuyen dong: "));
  Serial.print(MM_PER_ROTATION_2);
  Serial.println(F(" mm/vong"));
  Serial.print(F("  Steps/mm: "));
  Serial.println(STEPS_PER_MM_2);
  Serial.println();

  // Cấu hình chân Motor 1
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  digitalWrite(EN_PIN_1, LOW);  // LOW = enable motor (TMC2208 active LOW)

  // Cấu hình chân Motor 2
  pinMode(EN_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  digitalWrite(EN_PIN_2, LOW);  // LOW = enable motor
  
  // Cấu hình công tắc homing (INPUT_PULLUP - kích hoạt khi LOW)
  pinMode(HOME_SWITCH_Z, INPUT_PULLUP);
  pinMode(HOME_SWITCH_Y, INPUT_PULLUP);

  Serial.println(F("Pin config:"));
  Serial.println(F("  [MOTOR 1] DIR=6, STEP=5, EN=7"));
  Serial.println(F("  [MOTOR 2] DIR=4, STEP=3, EN=2"));
  Serial.println();

  // Cấu hình thông số cho Motor 1 với STEPS_PER_MM_1
  stepper1.setMaxSpeed(currentSpeed1 * STEPS_PER_MM_1);
  stepper1.setAcceleration(currentAccel1 * STEPS_PER_MM_1);

  // Cấu hình thông số cho Motor 2 với STEPS_PER_MM_2
  stepper2.setMaxSpeed(currentSpeed2 * STEPS_PER_MM_2);
  stepper2.setAcceleration(currentAccel2 * STEPS_PER_MM_2);

  // Thêm cả 2 stepper vào MultiStepper
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  Serial.println(F("Stepper Motor Ready!"));
  Serial.println(F("Dang cho du lieu tu ESP32 qua Serial (A1)..."));
  Serial.println(F("Format: z,y# (toa do tuyet doi mm)"));
  Serial.println(F("  HOME: H# (ve vi tri home 0,0)"));
  Serial.println(F("  Vi du: 100,50# -> Z=100mm, Y=50mm"));
  Serial.println();

  bufferIndex = 0;
  inputBuffer[0] = '\0';
  
  // Tự động thực hiện homing khi khởi động
  performHoming();
}

void loop() {
  // Không xử lý gì khi đang homing
  if (isHoming) {
    return;
  }
  
  // Chỉ đọc serial sau khi đã hoàn thành homing
  if (isHomed) {
    // Đọc dữ liệu từ ESP32 qua SoftwareSerial (A1)
    while (mySerial.available() > 0) {
      char c = mySerial.read();
      
      if (c == '#') {
        // Xử lý lệnh khi nhận ký tự kết thúc '#'
        if (bufferIndex > 0) {
          inputBuffer[bufferIndex] = '\0';  // Kết thúc chuỗi
          Serial.print(F("[ESP32] Received: "));
          Serial.println(inputBuffer);
          parseCoordinate(inputBuffer);
          bufferIndex = 0;  // Reset buffer
          inputBuffer[0] = '\0';
        }
      } else if (bufferIndex < 31) {  // Giới hạn 31 ký tự + 1 null terminator
        inputBuffer[bufferIndex++] = c;
      }
    }
    
    // Đọc dữ liệu từ Serial USB để debug (logic giống hệt)
    while (Serial.available() > 0) {
      char c = Serial.read();
      
      if (c == '#') {
        // Xử lý lệnh khi nhận ký tự kết thúc '#'
        if (bufferIndex > 0) {
          inputBuffer[bufferIndex] = '\0';  // Kết thúc chuỗi
          parseCoordinate(inputBuffer);
          bufferIndex = 0;  // Reset buffer
          inputBuffer[0] = '\0';
        }
      } else if (bufferIndex < 31) {  // Giới hạn 31 ký tự + 1 null terminator
        inputBuffer[bufferIndex++] = c;
      }
    }
  }

  // Xử lý Motor 2 (có thể chạy song song với Motor 1 trong chế độ realtime)
  if (isContinuous2) {
    // Chế độ quay liên tục với gia tốc - dùng run() thay vì runSpeed()
    stepper2.run();
  } else if (isMoving2 && stepper2.distanceToGo() != 0) {
    // Chế độ di chuyển theo khoảng cách
    stepper2.run();
  } else if (isMoving2 && stepper2.distanceToGo() == 0) {
    isMoving2 = false;
    float finalPos = stepper2.currentPosition() / (float)STEPS_PER_MM_2;
    Serial.println(F("\n=== [Motor 2] HOAN THANH ==="));
    Serial.print(F("Vi tri cuoi: "));
    Serial.print(finalPos, 2);
    Serial.println(F(" mm\n"));
    
    // Khi Motor 2 xong, kiểm tra xem Motor 1 có đang chờ không
    if (isPending1) {
      isPending1 = false;
      isMoving1 = true;
      Serial.println(F("[Motor 1] BAT DAU sau khi Motor 2 hoan thanh"));
    }
    
    // Kiểm tra nếu cần gửi DONE# và không có motor nào đang chạy
    if (needSendDone && !isMoving1 && !isPending1) {
      Serial.println("DONE#");  // Gửi về ESP32
      mySerial.println("DONE#"); // Gửi qua SoftwareSerial
      needSendDone = false;
      Serial.println(F("[INFO] Sent DONE# to ESP32 (after Motor 2)"));
    }
  }

  // Xử lý Motor 1 (trong chế độ realtime có thể chạy song song với Motor 2)
  // Chỉ chờ Motor 2 trong chế độ di chuyển tuyệt đối (khi isPending1 = true)
  if (!isPending1) {
    if (isContinuous1) {
      // Chế độ quay liên tục với gia tốc - dùng run() thay vì runSpeed()
      stepper1.run();
    } else if (isMoving1 && stepper1.distanceToGo() != 0) {
      // Chế độ di chuyển theo khoảng cách
      stepper1.run();
    } else if (isMoving1 && stepper1.distanceToGo() == 0) {
      isMoving1 = false;
      float finalPos = stepper1.currentPosition() / (float)STEPS_PER_MM_1;
      Serial.println(F("\n=== [Motor 1] HOAN THANH ==="));
      Serial.print(F("Vi tri cuoi: "));
      Serial.print(finalPos, 2);
      Serial.println(F(" mm\n"));
      
      // Kiểm tra nếu cả 2 motor đã xong và cần gửi DONE#
      if (needSendDone && !isMoving2 && !isPending1) {
        Serial.println("DONE#");  // Gửi về ESP32
        mySerial.println("DONE#"); // Gửi qua SoftwareSerial
        needSendDone = false;
        Serial.println(F("[INFO] Sent DONE# to ESP32"));
      }
    }
  }
}

// Hàm thực hiện homing (tìm vị trí 0,0)
void performHoming() {
  isHoming = true;  // Đánh dấu đang homing
  
  // Tắt SoftwareSerial để tránh interrupt xông đột với stepper timing
  mySerial.end();
  delay(50);
  
  // Clear buffer UART để tránh lệnh cũ từ ESP32
  while(Serial.available() > 0) {
    Serial.read();
  }
  bufferIndex = 0;
  inputBuffer[0] = '\0';
  
  // Dừng tất cả motor trước khi home
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  isMoving1 = false;
  isMoving2 = false;
  isContinuous1 = false;
  isContinuous2 = false;
  isPending1 = false;
  
  delay(100);  // Delay để motor dừng hoàn toàn
  
  Serial.println(F("\n=== BAT DAU HOMING ==="));
  
  // Home trục Z (Motor 2) trước
  Serial.println(F("[Truc Z] Dang tim home..."));
  stepper2.setMaxSpeed(currentSpeed2 * STEPS_PER_MM_2); // Tốc độ giữ nguyên 100%
  stepper2.setAcceleration(500 * STEPS_PER_MM_2); // Gia tốc phù hợp
  stepper2.move(-100000); // Di chuyển một khoảng lớn về phía âm
  
  while(digitalRead(HOME_SWITCH_Z) == HIGH && stepper2.distanceToGo() != 0) {
    // Chạy ngược cho đến khi chạm công tắc
    stepper2.run();
  }
  stepper2.stop();
  stepper2.setCurrentPosition(stepper2.currentPosition()); // Dừng ngay
  delay(100);
  
  // Lùi ra 1 chút
  stepper2.move(10 * STEPS_PER_MM_2);
  while(stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  
  // Set vị trí hiện tại = 0
  stepper2.setCurrentPosition(0);
  currentZ = 0;
  Serial.println(F("[Truc Z] Home thanh cong!"));
  
  // Nâng Z lên toạ độ 100mm sau khi home
  Serial.println(F("[Truc Z] Nang len 100mm..."));
  stepper2.moveTo(100 * STEPS_PER_MM_2);
  while(stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  currentZ = 100;
  Serial.println(F("[Truc Z] Da nang len 100mm!"));
  
  // Home trục Y (Motor 1)
  Serial.println(F("[Truc Y] Dang tim home..."));
  stepper1.setMaxSpeed(currentSpeed1 * STEPS_PER_MM_1); // Tốc độ giữ nguyên 100%
  stepper1.setAcceleration(500 * STEPS_PER_MM_1); // Gia tốc phù hợp
  stepper1.move(-100000); // Di chuyển một khoảng lớn về phía âm
  
  while(digitalRead(HOME_SWITCH_Y) == HIGH && stepper1.distanceToGo() != 0) {
    // Chạy ngược cho đến khi chạm công tắc
    stepper1.run();
  }
  stepper1.stop();
  stepper1.setCurrentPosition(stepper1.currentPosition()); // Dừng ngay
  delay(100);
  
  // Lùi ra 1 chút
  stepper1.move(10 * STEPS_PER_MM_1);
  while(stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
  
  // Set vị trí hiện tại = 0
  stepper1.setCurrentPosition(0);
  currentY = 0;
  Serial.println(F("[Truc Y] Home thanh cong!"));
  
  // Reset lại tốc độ và gia tốc về giá trị bình thường sau khi homing
  stepper1.setMaxSpeed(currentSpeed1 * STEPS_PER_MM_1);
  stepper1.setAcceleration(currentAccel1 * STEPS_PER_MM_1);
  stepper2.setMaxSpeed(currentSpeed2 * STEPS_PER_MM_2);
  stepper2.setAcceleration(currentAccel2 * STEPS_PER_MM_2);
  
  // Bật lại SoftwareSerial sau khi homing xong
  mySerial.begin(9600);
  delay(100);  // Delay cho serial khởi động ổn định
  
  // Clear buffer UART lần nữa sau khi homing xong
  while(mySerial.available() > 0) {
    mySerial.read();
  }
  while(Serial.available() > 0) {
    Serial.read();
  }
  bufferIndex = 0;
  inputBuffer[0] = '\0';
  
  isHomed = true;
  isHoming = false;  // Tắt cờ homing
  Serial.print(F("\n=== HOMING HOAN THANH (Z="));
  Serial.print(currentZ);
  Serial.print(F("mm, Y="));
  Serial.print(currentY);
  Serial.println(F("mm) ===\n"));
}

// Hàm parse toạ độ từ ESP32: z,y# hoặc H# (homing) hoặc Rz,y# (realtime)
void parseCoordinate(char* data) {
  // Kiểm tra lệnh HOME
  if (data[0] == 'H' || data[0] == 'h') {
    performHoming();
    return;
  }
  
  // Kiểm tra lệnh REALTIME (Rz,y#)
  if (data[0] == 'R' || data[0] == 'r') {
    Serial.println(F("[DEBUG] Realtime command detected"));
    handleRealtimeControl(data + 1); // Bỏ ký tự 'R' và xử lý phần còn lại
    return;
  }
  
  // Kiểm tra đã homing chưa
  if (!isHomed) {
    Serial.println(F("[LOI] Chua thuc hien HOMING! Gui 'H#' de homing truoc."));
    return;
  }
  
  // Parse toạ độ: z,y
  char* token = strtok(data, ",");
  if (token == NULL) return;
  targetZ = atof(token);
  
  token = strtok(NULL, ",");
  if (token == NULL) return;
  targetY = atof(token);
  
  // Kiểm tra tọa độ hợp lệ - không âm
  if (targetZ < 0) {
    Serial.print(F("[LOI] Toa do Z khong duoc am! Z="));
    Serial.println(targetZ);
    return;
  }
  if (targetY < 0) {
    Serial.print(F("[LOI] Toa do Y khong duoc am! Y="));
    Serial.println(targetY);
    return;
  }
  
  // Kiểm tra giới hạn hành trình
  if (targetZ > MAX_Z) {
    Serial.print(F("[LOI] Toa do Z vuot qua gioi han! Z="));
    Serial.print(targetZ);
    Serial.print(F(" mm > MAX_Z="));
    Serial.print(MAX_Z);
    Serial.println(F(" mm"));
    return;
  }
  if (targetY > MAX_Y) {
    Serial.print(F("[LOI] Toa do Y vuot qua gioi han! Y="));
    Serial.print(targetY);
    Serial.print(F(" mm > MAX_Y="));
    Serial.print(MAX_Y);
    Serial.println(F(" mm"));
    return;
  }
  
  // Debug
  Serial.print(F("Toa do muc tieu: Z="));
  Serial.print(targetZ);
  Serial.print(F(" mm, Y="));
  Serial.print(targetY);
  Serial.println(F(" mm"));
  
  // Tính khoảng cách cần di chuyển
  float deltaZ = targetZ - currentZ;  // Motor 2
  float deltaY = targetY - currentY;  // Motor 1
  
  Serial.print(F("Di chuyen: deltaZ="));
  Serial.print(deltaZ);
  Serial.print(F(" mm, deltaY="));
  Serial.print(deltaY);
  Serial.println(F(" mm"));
  
  // Xử lý Motor 2 (Trục Z) trước
  if (abs(deltaZ) > 0.01) {
    long stepsZ = deltaZ * STEPS_PER_MM_2;
    stepper2.move(stepsZ);
    isMoving2 = true;
    isContinuous2 = false;
  } else {
    isMoving2 = false;
  }
  
  // Xử lý Motor 1 (Trục Y)
  if (abs(deltaY) > 0.01) {
    long stepsY = deltaY * STEPS_PER_MM_1;
    stepper1.move(stepsY);
    
    if (isMoving2) {
      // Motor 2 (Z) đang chạy, Motor 1 (Y) chờ
      isPending1 = true;
      isMoving1 = false;
      Serial.println(F("[Truc Y] Cho truc Z hoan thanh"));
    } else {
      // Motor 2 không chạy, Motor 1 chạy ngay
      isMoving1 = true;
      isPending1 = false;
    }
    isContinuous1 = false;
  } else {
    isMoving1 = false;
    isPending1 = false;
  }
  
  // Cập nhật toạ độ hiện tại khi hoàn thành
  currentZ = targetZ;
  currentY = targetY;
  
  // Đánh dấu cần gửi DONE# sau khi cả 2 motor xong
  if (isMoving2 || isMoving1 || isPending1) {
    needSendDone = true;
  }
}

// Hàm xử lý điều khiển realtime: Rz,y# (z: 0=dừng, 1=tiến, 2=lùi)
void handleRealtimeControl(char* data) {
  Serial.print(F("[DEBUG] handleRealtimeControl data: "));
  Serial.println(data);
  
  // Parse lệnh realtime: z,y
  char* token = strtok(data, ",");
  if (token == NULL) {
    Serial.println(F("[DEBUG] Parse failed - no Z token"));
    return;
  }
  int cmdZ = atoi(token);  // 0=dừng, 1=tiến, 2=lùi
  
  token = strtok(NULL, ",");
  if (token == NULL) {
    Serial.println(F("[DEBUG] Parse failed - no Y token"));
    return;
  }
  int cmdY = atoi(token);  // 0=dừng, 1=tiến, 2=lùi
  
  Serial.print(F("[REALTIME] cmdZ="));
  Serial.print(cmdZ);
  Serial.print(F(", cmdY="));
  Serial.println(cmdY);
  
  // Xử lý Motor 2 (Trục Z) - Dùng move() + run() để có gia tốc
  if (cmdZ == 0) {
    // Dừng Motor 2 - Phải dừng hẳn
    if (isContinuous2 || isMoving2) {
      stepper2.stop();  // Dừng với gia tốc
      stepper2.setCurrentPosition(stepper2.currentPosition());
      isMoving2 = false;
      isContinuous2 = false;
      Serial.println(F("[Motor Z] STOPPED"));
    }
  } else if (cmdZ == 1) {
    // Tiến Motor 2 - chạy liên tục với gia tốc
    // Set vị trí đích rất xa (vô hạn) để motor chạy liên tục
    stepper2.move(1000000);  // 1 triệu steps = rất xa
    isMoving2 = true;
    isContinuous2 = true;
    Serial.print(F("[Motor Z] FORWARD continuous with acceleration, maxSpeed="));
    Serial.println(currentSpeed2 * STEPS_PER_MM_2);
  } else if (cmdZ == 2) {
    // Lùi Motor 2 - chạy liên tục với gia tốc
    stepper2.move(-1000000);  // -1 triệu steps = rất xa về phía âm
    
    isMoving2 = true;
    isContinuous2 = true;
    Serial.print(F("[Motor Z] BACKWARD continuous with acceleration, maxSpeed="));
    Serial.println(currentSpeed2 * STEPS_PER_MM_2);
  }
  
  // Xử lý Motor 1 (Trục Y) - Dùng move() + run() để có gia tốc
  if (cmdY == 0) {
    // Dừng Motor 1 - Phải dừng hẳn
    if (isContinuous1 || isMoving1) {
      stepper1.stop();  // Dừng với gia tốc
      stepper1.setCurrentPosition(stepper1.currentPosition());
      isMoving1 = false;
      isContinuous1 = false;
      isPending1 = false;
      Serial.println(F("[Motor Y] STOPPED"));
    }
  } else if (cmdY == 1) {
    // Tiến Motor 1 - chạy liên tục với gia tốc
    stepper1.move(1000000);  // 1 triệu steps = rất xa
    isMoving1 = true;
    isContinuous1 = true;
    isPending1 = false;
    Serial.print(F("[Motor Y] FORWARD continuous with acceleration, maxSpeed="));
    Serial.println(currentSpeed1 * STEPS_PER_MM_1);
  } else if (cmdY == 2) {
    // Lùi Motor 1 - chạy liên tục với gia tốc
    stepper1.move(-1000000);  // -1 triệu steps = rất xa về phía âm
    isMoving1 = true;
    isContinuous1 = true;
    isPending1 = false;
    Serial.print(F("[Motor Y] BACKWARD continuous with acceleration, maxSpeed="));
    Serial.println(currentSpeed1 * STEPS_PER_MM_1);
  }
}

