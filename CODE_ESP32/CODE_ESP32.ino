#include <PS2X_lib.h>
#include <ESP32Servo.h> 

// khai báo servo
Servo srv1;
Servo srv2;
#define SERVO1_PIN 12
#define SERVO2_PIN 13
// Khai báo đối tượng PS2X
PS2X ps2x;

// Định nghĩa các chân kết nối PS2 (ESP32 DevKit)
#define PS2_DAT 19  // Data (MISO)
#define PS2_CMD 23  // Command (MOSI)
#define PS2_SEL 5   // Attention (Select/CS)
#define PS2_CLK 18  // Clock (SCK)

// Serial1 để truyền dữ liệu sang Arduino Nano_1
// ESP32 Serial1: RX = GPIO4, TX = GPIO2
#define RXD1 4
#define TXD1 2

// Serial2 để truyền dữ liệu sang Arduino Nano_2
// ESP32 Serial2: RX = GPIO16, TX = GPIO17
#define RXD2 16
#define TXD2 17

// Động cơ DC - Driver L298N hoặc tương tự
#define DC_MOTOR_PWM 25  // PWM (tốc độ)
#define DC_MOTOR_IN1 26  // Hướng 1
#define DC_MOTOR_IN2 27  // Hướng 2

// Biến lưu trạng thái
int error = 0;
byte type = 0;
byte vibrate = 0;

// Mảng lưu dữ liệu điều khiển động cơ
float send_M1_M2[4] = {0, 0, 0, 0}; // {dir_L, rpm_L, dir_R, rpm_R} - cho Nano_2 (bánh xe)
float send_Stepper[4] = {0, 0, 0, 0}; // {dir_Step1, dist1, dir_Step2, dist2} - cho Nano_1 (stepper)

// Biến điều khiển servo
unsigned long servo1Time = 0;  // Thời điểm servo 1 hoạt động
unsigned long servo2Time = 0;  // Thời điểm servo 2 hoạt động
bool servo1Moving = false;     // Servo 1 đang chờ
bool servo2Moving = false;     // Servo 2 đang chờ
int targetAngle1 = 90;         // Góc mục tiêu servo 1
int targetAngle2 = 90;         // Góc mục tiêu servo 2
bool lastR1 = false;           // Trạng thái nút R1 trước
bool lastR2 = false;           // Trạng thái nút R2 trước

// Biến cho stepper realtime control
int lastCmdZ = 0;              // Lệnh Z trước đó
int lastCmdY = 0;              // Lệnh Y trước đó
int stopCounter = 0;           // Đếm số lần gửi stop
bool lastStart = false;        // Trạng thái nút START trước

// Biến cho PC control qua Serial (USB)
String pcCommandBuffer = "";  // Buffer lưu lệnh từ PC
bool pcCommandReady = false;   // Cờ đánh dấu nhận đủ lệnh
bool pcControlActive = false;  // Đang điều khiển từ PC
unsigned long pcStopTime = 0;  // Thời điểm nhận lệnh D# từ PC
bool waitingAfterPCStop = false; // Đang chờ 1s sau khi PC gửi D#

// Biến cho harvest automation
enum HarvestState {
  HARVEST_IDLE,       // Không làm gì
  HARVEST_WAIT_MOVE1, // Chờ Nano_1 di chuyển đến tọa độ dâu
  HARVEST_CUT,        // Thực hiện cắt dâu
  HARVEST_WAIT_MOVE2, // Chờ Nano_1 di chuyển về khay
  HARVEST_RELEASE     // Thả dâu
};

HarvestState harvestState = HARVEST_IDLE;
float currentZ = 0;  // Lưu tọa độ Z hiện tại để dùng cho về khay
String nano1Buffer = "";  // Buffer đọc từ Nano_1
bool nano1ResponseReady = false;

void setup() {
  Serial.begin(115200);  // Serial cho debug
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1); // Serial1 để truyền sang Nano_1
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Serial2 để truyền sang Nano_2
  
  pcCommandBuffer.reserve(50);  // Dự trữ bộ nhớ cho buffer PC
  nano1Buffer.reserve(20);       // Dự trữ bộ nhớ cho buffer Nano_1
  
  Serial.println("Khoi tao tay cam PS2...");
  Serial.println("PC Control: Gui T# de tien, D# de dung");
  
  delay(300);  // Delay để PS2 module khởi động
  
  // Khởi tạo PS2X controller
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  
  if(error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.println("pressures = false");
    Serial.println("rumble = false");
    Serial.println("Try out all the buttons!");
    
    type = ps2x.readType();
    switch(type) {
      case 0:
        Serial.println("Unknown Controller type found");
        break;
      case 1:
        Serial.println("DualShock Controller found");
        break;
      case 2:
        Serial.println("GuitarHero Controller found");
        break;
      case 3:
        Serial.println("Wireless Sony DualShock Controller found");
        break;
    }
  } else if(error == 1) {
    Serial.println("No controller found, check wiring");
  } else if(error == 2) {
    Serial.println("Controller found but not accepting commands");
  } else if(error == 3) {
    Serial.println("Controller refusing to enter Pressures mode");
  }
  	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  srv1.setPeriodHertz(50);// Standard 50hz servo
  srv1.attach(SERVO1_PIN, 500, 2400);   
  srv2.setPeriodHertz(50);// Standard 50hz servo
  srv2.attach(SERVO2_PIN, 500, 2400);   
  delay(50);
  srv1.write(90); // chuyen servo ve vi tri 90 degrees
  srv2.write(90); // chuyen servo ve vi tri 90 degrees
  delay(1000);
  
  // Khởi tạo động cơ DC
  pinMode(DC_MOTOR_PWM, OUTPUT);
  pinMode(DC_MOTOR_IN1, OUTPUT);
  pinMode(DC_MOTOR_IN2, OUTPUT);
  
  // Dừng động cơ ban đầu
  digitalWrite(DC_MOTOR_IN1, LOW);
  digitalWrite(DC_MOTOR_IN2, LOW);
  analogWrite(DC_MOTOR_PWM, 0);
  
  Serial.println("DC Motor initialized on pins 25-26-27");
}

void loop() {
  // Đọc lệnh từ PC qua Serial (USB)
  readPCCommand();
    // Đọc response từ Nano_1
  readNano1Response();
  
  // Xử lý harvest state machine
  handleHarvestSequence();
    // Kiểm tra nếu đang chờ sau khi PC gửi D#
  if (waitingAfterPCStop) {
    if (millis() - pcStopTime >= 1000) {
      // Đã chờ đủ 1s, cho phép PS2 gửi lệnh xuống Nano lại
      waitingAfterPCStop = false;
      Serial.println("[PC] 1s elapsed - PS2 control to Nano enabled");
    }
  }
  
  if(error == 1) {
    return; // Không đọc nếu không có tay cầm
  }
  
  // Luôn đọc PS2 (để có thể dùng emergency stop)
  read_PS2();
}

void readPCCommand() {
  // Đọc dữ liệu từ Serial (USB - PC)
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '#') {
      pcCommandReady = true;
      break;
    } else if (inChar == '\n' || inChar == '\r') {
      // Bỏ qua ký tự xuống dòng
      continue;
    } else {
      pcCommandBuffer += inChar;
      
      // Bảo vệ tràn buffer
      if (pcCommandBuffer.length() > 40) {
        pcCommandBuffer = "";
      }
    }
  }
  
  // Xử lý lệnh khi nhận đủ
  if (pcCommandReady) {
    pcCommandBuffer.trim();
    pcCommandBuffer.toUpperCase();
    
    Serial.print("[PC] Received: ");
    Serial.println(pcCommandBuffer);
    
    if (pcCommandBuffer == "T") {
      // Lệnh TIEN - bật PC control mode
      pcControlActive = true;
      waitingAfterPCStop = false;
      
      Serial.println("[PC] TIEN mode activated - PS2 disabled, continuous forward");
      
    } else if (pcCommandBuffer == "D") {
      // Lệnh DUNG - tắt PC control mode và gửi dừng
      pcControlActive = false;
      
      // Gửi lệnh dừng xuống Nano_2
      String wheelString = "0,0,0,0#";
      Serial2.print(wheelString);
      
      // Bắt đầu chờ 1s
      pcStopTime = millis();
      waitingAfterPCStop = true;
      
      Serial.println("[PC] DUNG - Sent stop, waiting 1s before PS2 control");
      
    } else if (pcCommandBuffer.startsWith("G")) {
      // Lệnh tọa độ Gz,y# - chuyển tiếp xuống Nano_1 và bắt đầu harvest sequence
      String coordStr = pcCommandBuffer.substring(1);  // Bỏ chữ "G"
      
      // Parse Z và Y
      int commaIndex = coordStr.indexOf(',');
      if (commaIndex > 0) {
        float z = coordStr.substring(0, commaIndex).toFloat();
        float y = coordStr.substring(commaIndex + 1).toFloat();
        
        // Lưu Z hiện tại để dùng cho về khay
        currentZ = z;
        
        // Tạo lệnh gửi xuống Nano_1 (format: z,y#)
        String cmd = String(z, 1) + "," + String(y, 1) + "#";
        Serial1.print(cmd);
        
        // Mở gắp ra trước khi bắt đầu (giống R2)
        performRelease();
        Serial.println("[AUTO] Opening gripper before movement");
        
        // Chuyển sang state chờ di chuyển
        harvestState = HARVEST_WAIT_MOVE1;
        
        Serial.print("[PC] COORD sent to Nano_1: ");
        Serial.print(cmd);
        Serial.print(" (Z=");
        Serial.print(z);
        Serial.print(", Y=");
        Serial.print(y);
        Serial.println(") - Waiting for DONE#");
      } else {
        Serial.println("[PC] Invalid coordinate format!");
      }
      
    } else {
      Serial.println("[PC] Unknown command: " + pcCommandBuffer);
    }
    
    // Reset buffer
    pcCommandBuffer = "";
    pcCommandReady = false;
  }
}

void readNano1Response() {
  // Đọc phản hồi từ Nano_1 (Serial1)
  while (Serial1.available() > 0) {
    char inChar = (char)Serial1.read();
    
    if (inChar == '#') {
      // Chỉ xử lý DONE# nếu đang ở chế độ auto harvest
      if (harvestState != HARVEST_IDLE) {
        nano1ResponseReady = true;
        Serial.print("[NANO_1] ");
        Serial.println(nano1Buffer + "#");  // Forward lên PC với format rõ ràng
      } else {
        // Chế độ thủ công - chỉ hiển thị, không điều khiển servo
        Serial.print("[NANO_1 Manual] ");
        Serial.println(nano1Buffer + "#");
      }
      break;
    } else if (inChar == '\n' || inChar == '\r') {
      continue;
    } else {
      nano1Buffer += inChar;
      
      if (nano1Buffer.length() > 20) {
        nano1Buffer = "";
      }
    }
  }
}

void handleHarvestSequence() {
  // Xử lý state machine cho harvest automation
  static unsigned long servoStartTime = 0;
  
  switch (harvestState) {
    case HARVEST_IDLE:
      // Không làm gì, chờ lệnh G từ PC
      break;
      
    case HARVEST_WAIT_MOVE1:
      // Chờ Nano_1 gửi DONE# sau khi di chuyển đến tọa độ dâu
      if (nano1ResponseReady) {
        nano1Buffer.trim();
        nano1Buffer.toUpperCase();
        
        Serial.print("[HARVEST] Received response: '");
        Serial.print(nano1Buffer);
        Serial.println("'");
        
        if (nano1Buffer == "DONE") {
          Serial.println("[HARVEST] Nano_1 reached target - Starting CUT");
          harvestState = HARVEST_CUT;
          
          // Bắt đầu cắt dâu (giống R1)
          performCut();
          servoStartTime = millis();
        } else {
          Serial.println("[HARVEST] WARNING: Expected DONE but got something else");
        }
        
        nano1Buffer = "";
        nano1ResponseReady = false;
      }
      break;
      
    case HARVEST_CUT:
      // Chờ servo cắt xong (2s: servo1 0s + servo2 1s + buffer 1s)
      if (millis() - servoStartTime >= 2500) {
        Serial.println("[HARVEST] Cut complete - Moving to tray");
        
        // Gửi lệnh di chuyển về khay: z_current,10#
        String cmd = String(currentZ, 1) + ",10#";
        Serial1.print(cmd);
        Serial.println("[HARVEST] Sent to Nano_1: " + cmd + " (move to tray)");
        
        harvestState = HARVEST_WAIT_MOVE2;
      }
      break;
      
    case HARVEST_WAIT_MOVE2:
      // Chờ Nano_1 gửi DONE# sau khi di chuyển về khay
      if (nano1ResponseReady) {
        nano1Buffer.trim();
        nano1Buffer.toUpperCase();
        
        Serial.print("[HARVEST] Received response: '");
        Serial.print(nano1Buffer);
        Serial.println("'");
        
        if (nano1Buffer == "DONE") {
          Serial.println("[HARVEST] Reached tray - Releasing strawberry");
          harvestState = HARVEST_RELEASE;
          
          // Bắt đầu thả dâu (giống R2)
          performRelease();
          servoStartTime = millis();
        } else {
          Serial.println("[HARVEST] WARNING: Expected DONE but got something else");
        }
        
        nano1Buffer = "";
        nano1ResponseReady = false;
      }
      break;
      
    case HARVEST_RELEASE:
      // Chờ servo thả xong (2s)
      if (millis() - servoStartTime >= 2500) {
        Serial.println("[HARVEST] ✅ HARVEST SEQUENCE COMPLETE!");
        
        // Mở gắp khi về vị trí chờ
        performRelease();
        
        // Gửi lệnh về vị trí chờ: Z=100mm, Y=10mm
        String waitCmd = "100,10#";
        Serial1.print(waitCmd);
        Serial.println("[HARVEST] Moving to wait position: Z=100mm, Y=10mm (gripper open)");
        
        // Gửi thông báo về PC
        Serial.println("HARVEST_DONE#");
        
        // Reset state
        harvestState = HARVEST_IDLE;
        currentZ = 0;
      }
      break;
  }
}

void performCut() {
  // Cắt dâu - giống như nhấn R1
  // Servo 1 đóng 81° ngay lập tức
  // Servo 2 đóng 75° sau 1s
  unsigned long currentTime = millis();
  
  targetAngle1 = 81;
  targetAngle2 = 75;
  servo1Time = currentTime;
  servo2Time = currentTime + 1000;
  servo1Moving = false;
  servo2Moving = true;
  
  srv1.write(targetAngle1);
  Serial.println("[CUT] Servo1=81° (immediate), Servo2=75° (after 1s)");
}

void performRelease() {
  // Thả dâu - giống như nhấn R2
  // Servo 2 mở 170° ngay lập tức
  // Servo 1 mở 170° sau 1s
  unsigned long currentTime = millis();
  
  targetAngle1 = 170;
  targetAngle2 = 170;
  servo1Time = currentTime + 1000;
  servo2Time = currentTime;
  servo1Moving = true;
  servo2Moving = false;
  
  srv2.write(targetAngle2);
  Serial.println("[RELEASE] Servo2=170° (immediate), Servo1=170° (after 1s)");
}

void handleServo() {
  unsigned long currentTime = millis();
  
  // Kiểm tra nút R1 (đóng 90 độ - Servo 1 trước) // Cắt, kẹp dây
  if(ps2x.Button(PSB_R1) && !lastR1) { 
    // Phát hiện nhấn R1 lần đầu
    targetAngle1 = 81;
    targetAngle2 = 75;
    servo1Time = currentTime;
    servo2Time = currentTime + 1000; // Servo 2 chờ 1s
    servo1Moving = false; // Servo 1 không cần chờ
    servo2Moving = true;  // Servo 2 chờ
    
    // Di chuyển servo 1 ngay
    srv1.write(targetAngle1);
    Serial.println("R1: Servo 1 -> 90 deg (ngay), Servo 2 cho 1s");
  }
  lastR1 = ps2x.Button(PSB_R1);
  
  // Kiểm tra nút R2 (mở 180 độ - Servo 2 trước) // Mở, thả dâu vào khay
  if(ps2x.Button(PSB_R2) && !lastR2) {
    // Phát hiện nhấn R2 lần đầu
    targetAngle1 = 170;
    targetAngle2 = 170;
    servo2Time = currentTime;
    servo1Time = currentTime + 1000; // Servo 1 chờ 1s
    servo1Moving = true;  // Servo 1 chờ
    servo2Moving = false; // Servo 2 không cần chờ
    
    // Di chuyển servo 2 ngay
    srv2.write(targetAngle2);
    Serial.println("R2: Servo 2 -> 180 deg (ngay), Servo 1 cho 1s");
  }
  lastR2 = ps2x.Button(PSB_R2);
  
  // Kiểm tra xem đã đến lúc di chuyển servo 1 chưa
  if(servo1Moving && currentTime >= servo1Time) {
    srv1.write(targetAngle1);
    servo1Moving = false;
    Serial.print("Servo 1 -> ");
    Serial.print(targetAngle1);
    Serial.println(" deg");
  }
  
  // Kiểm tra xem đã đến lúc di chuyển servo 2 chưa
  if(servo2Moving && currentTime >= servo2Time) {
    srv2.write(targetAngle2);
    servo2Moving = false;
    Serial.print("Servo 2 -> ");
    Serial.print(targetAngle2);
    Serial.println(" deg");
  }
}

void read_PS2() {
  // Đọc dữ liệu từ tay cầm
  ps2x.read_gamepad();
  
  // ===== NÚT EMERGENCY STOP (SELECT) =====
  if(ps2x.Button(PSB_SELECT)) {
    // Dừng khẩn cấp - tắt PC control và gửi STOP lên PC
    if (pcControlActive || waitingAfterPCStop) {
      pcControlActive = false;
      waitingAfterPCStop = false;
      
      // Gửi lệnh dừng xuống Nano_2
      String wheelString = "0,0,0,0#";
      Serial2.print(wheelString);
      
      // Gửi STOP lên PC
      Serial.println("STOP");
      
      Serial.println("[EMERGENCY] SELECT pressed - PC control stopped, sent STOP to PC");
      delay(500);  // Debounce
      return;
    }
  }
  
  // Xử lý servo với millis (không blocking)
  handleServo();
  
  // ===== ĐIỀU KHIỂN ĐỘNG CƠ DC =====
  // PSB_L1: Quay thuận (IN1=HIGH, IN2=LOW, PWM=50)
  if(ps2x.Button(PSB_L1)) {
    digitalWrite(DC_MOTOR_IN1, HIGH);
    digitalWrite(DC_MOTOR_IN2, LOW);
    analogWrite(DC_MOTOR_PWM, 130);
    Serial.println("[DC Motor] Quay thuan - PWM=100");
  }
  // PSB_L2: Dừng quay
  else if(ps2x.Button(PSB_L2)) {
    digitalWrite(DC_MOTOR_IN1, LOW);
    digitalWrite(DC_MOTOR_IN2, LOW);
    analogWrite(DC_MOTOR_PWM, 0);
    Serial.println("[DC Motor] Dung");
  }
  
  // Kiểm tra nút START để gửi lệnh HOME
  if(ps2x.Button(PSB_START) && !lastStart) {
    // Phát hiện nhấn START lần đầu
    Serial.println("[START] Gui lenh HOME (H#) den Nano_1...");
    Serial1.print("H#");  // Gửi lệnh home qua Serial1
    delay(100);  // Delay ngắn để đảm bảo lệnh được gửi
  }
  lastStart = ps2x.Button(PSB_START);
  
  // Khởi tạo giá trị mặc định (dừng)
  send_M1_M2[0] = 0; // dir_L = 0
  send_M1_M2[1] = 0; // rpm_L = 0
  send_M1_M2[2] = 0; // dir_R = 0
  send_M1_M2[3] = 0; // rpm_R = 0
  
  // Biến lưu lệnh Realtime cho Stepper: Rz,y#
  // z: 0=dừng, 1=tiến, 2=lùi (Motor 2 - Trục Z)
  // y: 0=dừng, 1=tiến, 2=lùi (Motor 1 - Trục Y)
  int cmdZ = 0;
  int cmdY = 0;
  bool needSendStepper = false;
  
  // Kiểm tra các nút điều khiển Stepper Motor (Realtime mode)
  // PSB_TRIANGLE (GREEN) = Y tiến (Motor 1)
  // PSB_CIRCLE (RED) = Y lùi (Motor 1)
  // PSB_CROSS (BLUE) = Z tiến (Motor 2)
  // PSB_SQUARE (PINK) = Z lùi (Motor 2)
  if(ps2x.Button(PSB_CIRCLE)) {
    // GREEN: Motor 1 (Y) tiến
    cmdY = 1;
    Serial.println("Command: Y TIEN");
  }
  if(ps2x.Button(PSB_SQUARE)) {
    // RED: Motor 1 (Y) lùi
    cmdY = 2;
    Serial.println("Command: Y LUI");
  }
  if(ps2x.Button(PSB_TRIANGLE)) {
    // BLUE: Motor 2 (Z) tiến
    cmdZ = 1;
    Serial.println("Command: Z TIEN");
  }
  if(ps2x.Button(PSB_CROSS)) {
    // PINK: Motor 2 (Z) lùi
    cmdZ = 2;
    Serial.println("Command: Z LUI");
  }
  
  // Kiểm tra xem có thay đổi lệnh không
  if (cmdZ != lastCmdZ || cmdY != lastCmdY) {
    // Có thay đổi -> gửi lệnh
    needSendStepper = true;
    lastCmdZ = cmdZ;
    lastCmdY = cmdY;
    
    // Nếu đang bấm (không phải dừng) -> reset stopCounter
    if (cmdZ != 0 || cmdY != 0) {
      stopCounter = 0;
    }
  } else if (cmdZ == 0 && cmdY == 0 && stopCounter < 2) {
    // Đang ở trạng thái dừng và chưa gửi đủ 2 lần -> gửi tiếp
    needSendStepper = true;
    stopCounter++;
  }
  
  // Kiểm tra các nút D-pad cho bánh xe (gửi cho Nano_2)
  if(ps2x.Button(PSB_PAD_UP)) {
    // Tiến: cả 2 động cơ tiến (dir=1), rpm=50
    send_M1_M2[0] = 1; // dir_L = 1
    send_M1_M2[1] = 50; // rpm_L = 50
    send_M1_M2[2] = 1; // dir_R = 1
    send_M1_M2[3] = 50; // rpm_R = 50
    Serial.println("Command: TIEN");
  }
  else if(ps2x.Button(PSB_PAD_DOWN)) {
    // Lùi: cả 2 động cơ lùi (dir=2), rpm=50
    send_M1_M2[0] = 2; // dir_L = 2
    send_M1_M2[1] = 50; // rpm_L = 50
    send_M1_M2[2] = 2; // dir_R = 2
    send_M1_M2[3] = 50; // rpm_R = 50
    Serial.println("Command: LUI");
  }
  else if(ps2x.Button(PSB_PAD_LEFT)) {
    // Quay trái: L lùi, R tiến
    send_M1_M2[0] = 2; // dir_L = 2 (lùi)
    send_M1_M2[1] = 50; // rpm_L = 50
    send_M1_M2[2] = 1; // dir_R = 1 (tiến)
    send_M1_M2[3] = 50; // rpm_R = 50
    Serial.println("Command: TRAI");
  }
  else if(ps2x.Button(PSB_PAD_RIGHT)) {
    // Quay phải: L tiến, R lùi
    send_M1_M2[0] = 1; // dir_L = 1 (tiến)
    send_M1_M2[1] = 50; // rpm_L = 50
    send_M1_M2[2] = 2; // dir_R = 2 (lùi)
    send_M1_M2[3] = 50; // rpm_R = 50
    Serial.println("Command: PHAI");
  }
  
  // ===== GỬI LỆNH XUỐNG NANO =====
  // Nếu PC control active hoặc đang chờ sau PC stop, không gửi lệnh từ PS2
  if (pcControlActive) {
    // PC đang điều khiển - gửi lệnh tiến liên tục
    String wheelString = "1,50,1,50#";  // Tiến, RPM=50
    Serial2.print(wheelString);
    // Không gửi lệnh stepper
    delay(50);
    return;
  }
  
  if (waitingAfterPCStop) {
    // Đang chờ 1s sau khi PC dừng - không gửi gì cả
    return;
  }
  
  // Chỉ gửi lệnh stepper khi cần thiết (UART1 - logic mới)
  if (needSendStepper) {
    String stepperString = "R" + String(cmdZ) + "," + String(cmdY) + "#";
    Serial1.print(stepperString); // Gửi qua Serial1 sang Nano_1 (Stepper)
    Serial.print("Sent to Nano_1 (Stepper): ");
    Serial.println(stepperString);
  }
  
  // Gửi lệnh wheel liên tục (UART2 - logic cũ)
  String wheelString = String((int)send_M1_M2[0]) + "," + 
                       String((int)send_M1_M2[1]) + "," + 
                       String((int)send_M1_M2[2]) + "," + 
                       String((int)send_M1_M2[3]) + "#";
  Serial2.print(wheelString);   // Gửi qua Serial2 sang Nano_2 (Bánh xe)
 // Serial.print("Sent to Nano_2 (Wheel): ");
 // Serial.println(wheelString);
  
  delay(50);  // Delay 50ms để tránh gửi quá nhanh
}
