#include <queue>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ESP32Servo.h>
#include <math.h>  // สำหรับฟังก์ชันทางคณิตศาสตร์
#include "RobotArmIK.h"
#include <EEPROM.h>
#include "WebPages.h"  // Include the web pages header
const char* ssid = "Terk_2.4GHz";      // ชื่อ Wi-Fi ของคุณ
const char* password = "08110171188";  // รหัสผ่าน Wi-Fi ของคุณ

struct Command {
  String type;
  int theta1;
  int theta2;
  int theta3;
  int speed;
  int acceleration;
};

// สร้างคิวสำหรับจัดเก็บคำสั่ง
std::queue<Command> commandQueue;
// สร้างวัตถุ WebServer (ใช้พอร์ต 80)
WebServer server(80);
int len = 0;
int wifi_connecy_count = 0;
// สร้างวัตถุมอเตอร์และเซอร์โวเหมือนในโค้ดเดิม
AccelStepper stepper1(1, 17, 16);  // ข้อต่อ 1 หมุนรอบแกน Z
AccelStepper stepper2(1, 18, 5);   // ข้อต่อ 2
AccelStepper stepper3(1, 21, 19);  // ข้อต่อ 3

// สร้างวัตถุ MultiStepper สำหรับการควบคุมแบบประสานงาน
MultiStepper multiStepper;

Servo gripperServo;
// กำหนด Limit Switch
#define limitSwitch1 26    // Limit switch สำหรับมอเตอร์ 1
#define limitSwitch2 25    // Limit switch สำหรับมอเตอร์ 2
#define limitSwitch3 33    // Limit switch สำหรับมอเตอร์ 3
#define DEBOUNCE_DELAY 10  // เวลาในการหน่วง debounce (50 มิลลิวินาที)

// ประกาศ Timer
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define MAX_STEPS 14  // จำนวนขั้นตอนสูงสุดที่ต้องการสำหรับ trajectory

// ตัวแปรจัดเก็บตำแหน่งมุมของแต่ละขั้นตอน
int theta1Steps[MAX_STEPS];
int theta2Steps[MAX_STEPS];
int theta3Steps[MAX_STEPS];

int currentStep = 0;    // ขั้นตอนปัจจุบัน
int totalSteps = 0;     // จำนวนขั้นตอนทั้งหมด
bool isMoving = false;  // สถานะของการเคลื่อนที่

// ความยาวของข้อต่อ (สมมุติ)
double L1 = 135;          // ความยาวของข้อต่อแรก L1 = 135 มม.
double L2 = 147;          // ความยาวของข้อต่อที่สอง L2 = 147 มม.
double baseHeight = 156.5;  // ความสูงจากพื้น 156 มม.

int stepper1Position, stepper2Position, stepper3Position;
float theta1AngleToSteps = 2.8778;
float theta2AngleToSteps = 2.8778;
float phiAngleToSteps = 23.222222;

// ตัวแปรเก็บค่าที่ถูกส่งไปล่าสุด
int lastTheta1 = 250;
int lastTheta2 = 210;
int lastTheta3 = 50;

int lastTheta1_joint = 0;
int lastTheta2_joint = 0;
int lastTheta3_joint = 0;

int targetTheta1_i = 0;
int targetTheta2_i = 0;
int targetTheta3_i = 0;

int lastTheta1_home = 5;
int lastTheta2_home = 32;
int lastTheta3_home = 154;
int lastGripper = 180;
int lastSpeed = 100;
int lastAcceleration = 500;

// ตัวแปรสำหรับตำแหน่งปลายแขนกล (forward kinematics result)
double endEffectorX = 0;
double endEffectorY = 0;
double endEffectorZ = 0;  // เพิ่มแกน Z
g_Code cmd;
RobotArmIK robotArmIK(L1, L2, baseHeight, 125, 125, 420, 50);

String Arm_mode = "JOINT";
String savedSSID = "";
String savedPassword = "";

String content = "";
String state = "";
bool isFirstTime = false;  // เช็คว่าคือการเริ่มต้นครั้งแรกหรือไม่
char data[32];

unsigned char k;

// API Response Helper Functions
void sendApiResponse(int statusCode, String status, String message, String data = "{}") {
  String response = "{";
  response += "\"status\":\"" + status + "\",";
  response += "\"message\":\"" + message + "\",";
  response += "\"data\":" + data + ",";
  response += "\"timestamp\":" + String(millis());
  response += "}";
  
  server.sendHeader("Content-Type", "application/json");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  server.send(statusCode, "application/json", response);
}

void sendApiError(String message, int statusCode = 400) {
  sendApiResponse(statusCode, "error", message);
}

void sendApiSuccess(String message, String data = "{}") {
  sendApiResponse(200, "success", message, data);
}



String ip = "";
String gateway = "";
String subnet = "";
String urlMain = "http://183.88.230.236:12000";
int delay_time;

// ตั้งค่า flag เพื่อตรวจสอบการทำงานของ Timer
volatile bool timerFlag = false;

// ฟังก์ชัน Timer ISR
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}
// ฟังก์ชัน debounce สำหรับ Limit Switch
bool debounce(int pin) {
  bool state = digitalRead(pin);
  delay(DEBOUNCE_DELAY);  // รอช่วงเวลาหน่วง
  bool newState = digitalRead(pin);
  return state == newState ? state : HIGH;  // ตรวจสอบว่าค่าที่อ่านได้เป็นค่าเดิมหรือไม่
}

void setStepperSpeedAndAcceleration(int speed, int acceleration) {
  stepper1.setMaxSpeed(speed);
  stepper1.setAcceleration(acceleration);
  stepper2.setMaxSpeed(speed);
  stepper2.setAcceleration(acceleration);
  stepper3.setMaxSpeed(speed);
  stepper3.setAcceleration(acceleration);
}

// ฟังก์ชันสำหรับการเคลื่อนที่แบบประสานงานด้วย MultiStepper
void moveToPositionsCoordinated(long pos1, long pos2, long pos3, int speed, int acceleration) {
  // ตั้งค่าความเร็วและความเร่ง
  setStepperSpeedAndAcceleration(speed, acceleration);
  
  // กำหนดตำแหน่งเป้าหมาย
  // แปลงมุมเป็นสเต็ปและเคลื่อนมอเตอร์
  

  long positions[3];
  positions[0] = pos1;
  positions[1] = pos2;
  positions[2] = pos3;
  
  Serial.println("--- MultiStepper Movement Started ---");
  Serial.print("Target positions: ");
  Serial.print(pos1); Serial.print(", ");
  Serial.print(pos2); Serial.print(", ");
  Serial.println(pos3);
  
  // เคลื่อนที่แบบประสานงาน
  multiStepper.moveTo(positions);
  
  // รันมอเตอร์จนกว่าจะถึงตำแหน่งเป้าหมาย
  while (multiStepper.run()) {
    // ตรวจสอบ limit switch ระหว่างการเคลื่อนที่
    if (checkLimitSwitch()) {
      Serial.println("Movement stopped due to limit switch!");
     // break;
    }
  }
  
  Serial.println("--- MultiStepper Movement Completed ---");
  printCurrentPositions();
}

// ฟังก์ชันสำหรับแสดงตำแหน่งปัจจุบันของมอเตอร์ทั้งหมด
void printCurrentPositions() {
  Serial.print("Current positions: ");
  Serial.print("Stepper1: "); Serial.print(stepper1.currentPosition());
  Serial.print(", Stepper2: "); Serial.print(stepper2.currentPosition());
  Serial.print(", Stepper3: "); Serial.println(stepper3.currentPosition());
}
// ฟังก์ชันสำหรับการตั้งค่าตำแหน่งเริ่มต้นโดยใช้ Limit Switch พร้อม debounce
void setZero() {

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  // กำหนดทิศทางการเคลื่อนที่ให้เคลื่อนไปยัง Limit Switch
  stepper1.setSpeed(-2000);  // เคลื่อนที่ไปทิศทางตรงข้ามเพื่อหาจุดเริ่มต้น
  // stepper1.setAcceleration(500);
  stepper2.setSpeed(-2000);
  // stepper2.setAcceleration(500);
  stepper3.setSpeed(-2000);
  // stepper3.setAcceleration(500);
  while (debounce(limitSwitch1) == HIGH) {
    stepper1.runSpeed();
  }
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // รันมอเตอร์จนกว่าจะถึง Limit Switch (พร้อม debounce)
  while (debounce(limitSwitch2) == HIGH) {
    stepper2.runSpeed();
  }
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

  while (debounce(limitSwitch3) == HIGH) {
    stepper3.runSpeed();
  }
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

  Serial.println("Set Zero: มอเตอร์ทั้งหมดถูกตั้งค่าที่ตำแหน่ง 0");
}
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
bool checkLimitSwitch() {
  bool limit1 = digitalRead(limitSwitch1) == LOW;  // LOW เมื่อถูกกด
  bool limit2 = digitalRead(limitSwitch2) == LOW;
  bool limit3 = digitalRead(limitSwitch3) == LOW;

  // หากมี Limit Switch ใดถูกกด, ให้หยุดการเคลื่อนที่
  if (limit1 || limit2 || limit3) {
    Serial.println("Limit switch triggered! Motion stopped.");
    return true;  // มี Limit Switch ถูกกด
  }
  return false;  // ไม่มี Limit Switch ถูกกด
}

float lerp(float start, float end, float t) {
  return start + (end - start) * t;
}
// ฟังก์ชัน Trajectory Planning
void trajectoryPlanning(int targetTheta1, int targetTheta2, int targetTheta3, int duration) {

  totalSteps = MAX_STEPS;
  float timePerStep = (float)duration / totalSteps;

  // คำนวณตำแหน่งมุมในแต่ละขั้นตอน
  for (int i = 0; i < totalSteps; ++i) {
    float t = (float)i / totalSteps;

    // คำนวณมุมปัจจุบันโดยใช้ Linear Interpolation
    int currentTheta1 = lerp(lastTheta1, targetTheta1, t);
    int currentTheta2 = lerp(lastTheta2, targetTheta2, t);
    int currentTheta3 = lerp(lastTheta3, targetTheta3, t);

    // เรียกใช้ฟังก์ชัน runIK เพื่อคำนวณตำแหน่ง x, y, z ที่ต้องการ
    g_Code stepCmd = robotArmIK.runIK(currentTheta1, currentTheta2, currentTheta3, cmd);

    // จัดเก็บค่าที่คำนวณได้
    theta1Steps[i] = stepCmd.z * theta1AngleToSteps;
    theta2Steps[i] = stepCmd.y * theta2AngleToSteps;
    theta3Steps[i] = stepCmd.x * phiAngleToSteps;
    Command cmd;
    cmd.type = "move";
    cmd.theta1 = theta1Steps[i];
    cmd.theta2 = theta2Steps[i];
    cmd.theta3 = theta3Steps[i];
    cmd.speed = lastSpeed;
    cmd.acceleration = lastAcceleration;
    commandQueue.push(cmd);
  }

  // ตั้งค่าเริ่มต้นสำหรับการเคลื่อนที่
  currentStep = 0;
  isMoving = true;
  lastTheta1 = targetTheta1;
  lastTheta2 = targetTheta2;
  lastTheta3 = targetTheta3;
}

void Planning(int targetTheta1, int targetTheta2, int targetTheta3, int duration) {

  Command cmd;
  cmd.type = "move";
  cmd.theta1 = targetTheta1 * theta1AngleToSteps;
  cmd.theta2 = targetTheta2 * theta2AngleToSteps;
  cmd.theta3 = targetTheta3 * phiAngleToSteps;
  cmd.speed = lastSpeed;
  cmd.acceleration = lastAcceleration;
  commandQueue.push(cmd);


  // ตั้งค่าเริ่มต้นสำหรับการเคลื่อนที่
  currentStep = 0;
  isMoving = true;
  lastTheta1_joint = targetTheta1;
  lastTheta2_joint = targetTheta2;
  lastTheta3_joint = targetTheta3;
}
void handleSetup() {
  String html = generateSetupPage();
  server.send(200, "text/html", html);
}

void executeCommand(Command cmd) {
  // Set speed and acceleration
  stepper1.setMaxSpeed(cmd.speed);
  stepper1.setAcceleration(cmd.acceleration);
  stepper2.setMaxSpeed(cmd.speed);
  stepper2.setAcceleration(cmd.acceleration);
  stepper3.setMaxSpeed(cmd.speed * 10);
  stepper3.setAcceleration(cmd.acceleration );

  // Set target positions for coordinated movement
  long positions[3];
  positions[0] = cmd.theta1;  // stepper1 target
  positions[1] = cmd.theta2;  // stepper2 target
  positions[2] = cmd.theta3;  // stepper3 target
  
  // Move to target positions using MultiStepper for synchronized movement
  multiStepper.moveTo(positions);
  
  // Run motors until all reach their targets
  while (multiStepper.run()) {
    // Check limit switches during movement for safety
    if (checkLimitSwitch()) {
      Serial.println("Movement stopped due to limit switch!");
     // break;
    }
  }
}
void loadSettingsFromEEPROM() {
  theta1AngleToSteps = EEPROM.readFloat(0);
  theta2AngleToSteps = EEPROM.readFloat(4);
  phiAngleToSteps = EEPROM.readFloat(8);
  L1 = EEPROM.readFloat(12);
  L2 = EEPROM.readFloat(16);
  baseHeight = EEPROM.readFloat(20);

  // ตรวจสอบว่ามีการตั้งค่าไว้แล้วหรือไม่ ถ้าไม่ให้ใช้ค่าที่ตั้งไว้ในโค้ด
  if (theta1AngleToSteps == 0)
    theta1AngleToSteps = 77.222222;
  if (theta2AngleToSteps == 0)
    theta2AngleToSteps = 77.222222;
  if (phiAngleToSteps == 0)
    phiAngleToSteps = 23.222222;
  if (L1 == 0)
    L1 = 135;
  if (L2 == 0)
    L2 = 157;
  if (baseHeight == 0)
    baseHeight = 156;
}
void handleSaveWiFi() {
  savedSSID = server.arg("ssid");
  savedPassword = server.arg("password");
  ip = server.arg("ip");
  gateway = server.arg("gateway");
  subnet = server.arg("subnet");

  if (savedSSID.length() > 0 && savedPassword.length() > 0) {
    // บันทึก SSID และ Password
    writeStringToEEPROM(0, savedSSID);
    writeStringToEEPROM(32, savedPassword);
    EEPROM.commit();

    // บันทึก IP Address, Gateway และ Subnet
    writeStringToEEPROM(64, ip);
    writeStringToEEPROM(96, gateway);
    writeStringToEEPROM(128, subnet);
    EEPROM.commit();

    server.send(200, "text/html", "Settings saved. Restarting...");
    delay(2000);
    ESP.restart();
  } else {
    server.send(200, "text/html", "Please provide all required fields.");
  }
}
// ฟังก์ชันสำหรับแสดงหน้าเว็บพร้อมค่าสุดท้ายที่ผู้ใช้ส่ง
void handleRoot() {
  String html = generateRobotControlPage(lastTheta1, lastTheta2, lastTheta3, 
                                        lastGripper, lastSpeed, lastAcceleration,
                                        endEffectorX, endEffectorY, endEffectorZ);
  server.send(200, "text/html", html);
}


void handlejoint() {
  String html = generateJointControlPage(lastTheta1_joint, lastTheta2_joint, lastTheta3_joint,
                                        lastGripper, lastSpeed, lastAcceleration,
                                        endEffectorX, endEffectorY, endEffectorZ);
  server.send(200, "text/html", html);
}
void handleGcodePage() {
  String html = generateGcodePage();
  server.send(200, "text/html", html);
}

// Handler สำหรับรับข้อมูล G-code ที่ส่งมาจากฟอร์ม
void handleGcodeSubmit() {
  String gcode = server.arg("gcode");
  Serial.println("------G-code Received------");
  Serial.println(gcode);
  // รองรับหลายบรรทัด X100Y100Z100
  String resultHtml = "<html><body><h1>G-code Received</h1><pre>" + gcode + "</pre><hr><ul>";
  int lineStart = 0;
  while (lineStart < gcode.length()) {
    int lineEnd = gcode.indexOf('\n', lineStart);
    if (lineEnd == -1) lineEnd = gcode.length();
    String line = gcode.substring(lineStart, lineEnd);
    line.trim();
    if (line.length() > 0) {
      String type;
      int xVal = 0, yVal = 0, zVal = 0, delayVal = 0;
      parseGcodeLine(line, type, xVal, yVal, zVal, delayVal);

      if (type == "XYZ") {
        trajectoryPlanning(xVal, yVal, zVal, delayVal);
        Serial.print("X: ");
        Serial.print(xVal);
        Serial.print(", Y: ");
        Serial.print(yVal);
        Serial.print(", Z: ");
        Serial.println(zVal);
        resultHtml += "<li>" + line + " &rarr; X: " + String(xVal) + ", Y: " + String(yVal) + ", Z: " + String(zVal) + "</li>";
      } else if (type == "GPON") {
        Command cmd;
        cmd.type = "Gripper";
        cmd.theta1 = 0;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        Serial.println("Gripper ON");
        resultHtml += "<li>" + line + " &rarr; Gripper ON</li>";
      } else if (type == "GPOFF") {
        Command cmd;
        cmd.type = "Gripper";
        cmd.theta1 = 180;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        Serial.println("Gripper OFF");
        resultHtml += "<li>" + line + " &rarr; Gripper OFF</li>";
      } else if (type == "DELAY") {
        Command cmd;
        cmd.type = "delay";
        cmd.theta1 = delayVal;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        Serial.print("Delay: ");
        Serial.println(delayVal);
        resultHtml += "<li>" + line + " &rarr; Delay " + String(delayVal) + " ms</li>";
      } else {
        resultHtml += "<li>" + line + " &rarr; Unknown</li>";
      }
      // ฟังก์ชันแยกประเภทคำสั่ง G-code: XYZ, GPON, GPOFF, DELAY
    }
    lineStart = lineEnd + 1;
  }
  resultHtml += "</ul><a href='/gcode'>Back</a></body></html>";
  server.send(200, "text/html", resultHtml);
}


void serial() {
  if (Serial.available()) {
    content = Serial.readString();  // อ่านข้อมูลจาก Processing
    Serial.println(content);
    // รองรับหลายบรรทัด G-code เช่นเดียวกับหน้าเว็บ
    int lineStart = 0;
    while (lineStart < content.length()) {
      int lineEnd = content.indexOf('\n', lineStart);
      if (lineEnd == -1) lineEnd = content.length();
      String line = content.substring(lineStart, lineEnd);
      line.trim();
      if (line.length() > 0) {
        String type;
        int xVal = 0, yVal = 0, zVal = 0, delayVal = 0;
        parseGcodeLine(line, type, xVal, yVal, zVal, delayVal);

        if (type == "XYZ") {
          Serial.print("[SERIAL] X: ");
          Serial.print(xVal);
          Serial.print(", Y: ");
          Serial.print(yVal);
          Serial.print(", Z: ");
          Serial.println(zVal);
          trajectoryPlanning(xVal, yVal, zVal, delayVal);
        } else if (type == "GPON") {
          Command cmd;
          cmd.type = "Gripper";
          cmd.theta1 = 0;
          cmd.theta2 = 0;
          cmd.theta3 = 0;
          cmd.speed = 0;
          cmd.acceleration = 0;
          commandQueue.push(cmd);
          Serial.println("[SERIAL] Gripper ON");
        } else if (type == "GPOFF") {
          Command cmd;
          cmd.type = "Gripper";
          cmd.theta1 = 180;
          cmd.theta2 = 0;
          cmd.theta3 = 0;
          cmd.speed = 0;
          cmd.acceleration = 0;
          commandQueue.push(cmd);
          Serial.println("[SERIAL] Gripper OFF");
        } else if (type == "DELAY") {
          Command cmd;
          cmd.type = "delay";
          cmd.theta1 = delayVal;
          cmd.theta2 = 0;
          cmd.theta3 = 0;
          cmd.speed = 0;
          cmd.acceleration = 0;
          commandQueue.push(cmd);
          Serial.print("[SERIAL] Delay: ");
          Serial.println(delayVal);
        } else {
          Serial.println("[SERIAL] Unknown command");
        }
      }
      lineStart = lineEnd + 1;
    }
  }
}
void parseGcodeLine(const String& str, String& type, int& x, int& y, int& z, int& delayVal) {
  String s = str;
  s.trim();
  x = y = z = delayVal = 0;
  if (s.startsWith("X") && s.indexOf('Y') != -1 && s.indexOf('Z') != -1) {
    int xIdx = s.indexOf('X');
    int yIdx = s.indexOf('Y');
    int zIdx = s.indexOf('Z');
    if (xIdx != -1 && yIdx != -1 && zIdx != -1 && xIdx < yIdx && yIdx < zIdx) {
      String xStr = s.substring(xIdx + 1, yIdx);
      String yStr = s.substring(yIdx + 1, zIdx);
      String zStr = s.substring(zIdx + 1);
      x = xStr.toInt();
      y = yStr.toInt();
      z = zStr.toInt();
      type = "XYZ";
      return;
    }
  }
  if (s.equalsIgnoreCase("GPON")) {
    type = "GPON";
    return;
  }
  if (s.equalsIgnoreCase("GPOFF")) {
    type = "GPOFF";
    return;
  }
  if (s.startsWith("DELAY")) {
    String d = s.substring(5);
    d.trim();
    delayVal = d.toInt();
    type = "DELAY";
    return;
  }
  type = "UNKNOWN";
}

void parseXYZString(const String& str, int& x, int& y, int& z) {
  int xIdx = str.indexOf('X');
  int yIdx = str.indexOf('Y');
  int zIdx = str.indexOf('Z');
  if (xIdx != -1 && yIdx != -1 && zIdx != -1 && xIdx < yIdx && yIdx < zIdx) {
    String xStr = str.substring(xIdx + 1, yIdx);
    String yStr = str.substring(yIdx + 1, zIdx);
    String zStr = str.substring(zIdx + 1);
    x = xStr.toInt();
    y = yStr.toInt();
    z = zStr.toInt();
  } else {
    x = y = z = 0;
  }
}
void handleMain() {
  String html = generateMainPage(urlMain, ip);
  server.send(200, "text/html", html);
}
// ฟังก์ชันการแสดงหน้าเว็บสำหรับตั้งค่า
void handleSetupPage() {
  String html = generateConfigPage(theta1AngleToSteps, theta2AngleToSteps, phiAngleToSteps,
                                  L1, L2, baseHeight);
  server.send(200, "text/html", html);
}


void handleOta() {
  String html = generateOtaPage();
  server.send(200, "text/html", html);
}
void handleSaveSetupArm() {
  // อ่านค่าจากฟอร์ม
  theta1AngleToSteps = server.arg("theta1AngleToSteps").toFloat();
  theta2AngleToSteps = server.arg("theta2AngleToSteps").toFloat();
  phiAngleToSteps = server.arg("phiAngleToSteps").toFloat();
  L1 = server.arg("L1").toFloat();
  L2 = server.arg("L2").toFloat();
  baseHeight = server.arg("baseHeight").toFloat();
  String ip = server.arg("ip");
  String gateway = server.arg("gateway");
  String subnet = server.arg("subnet");

  // บันทึกค่าลงใน EEPROM
  EEPROM.writeFloat(0, theta1AngleToSteps);
  EEPROM.writeFloat(4, theta2AngleToSteps);
  EEPROM.writeFloat(8, phiAngleToSteps);
  EEPROM.writeFloat(12, L1);
  EEPROM.writeFloat(16, L2);
  EEPROM.writeFloat(20, baseHeight);
  writeStringToEEPROM(64, ip);
  writeStringToEEPROM(96, gateway);
  writeStringToEEPROM(128, subnet);
  EEPROM.commit();

  server.send(200, "text/html", "<html><body><h1>Settings Saved</h1><a href=\"/setup\">Go Back</a></body></html>");
}

void setHomePosition() {
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // เคลื่อนที่ไปยังตำแหน่ง home แบบประสานงาน
  moveToPositionsCoordinated(
    lastTheta1_home * theta1AngleToSteps,
    lastTheta2_home * theta2AngleToSteps,
    lastTheta3_home * phiAngleToSteps,
    1000, 200
  );
  
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // ควบคุม Gripper
  gripperServo.write(lastGripper);
}


void setHomeValue() {
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  lastTheta1_joint = 0;
  lastTheta2_joint = 0;
  lastTheta3_joint = 0;
  server.send(200, "text/plain", "Set home value success");
}

void goHomeValue() {
  Planning(0, 0, 0, 0);
  lastTheta1_joint = 0;
  lastTheta2_joint = 0;
  lastTheta3_joint = 0;
  server.send(200, "text/plain", "Set home value success");
}
void setHomeArm() {
  Planning(0, 0, 0, 0);
  lastTheta1_joint = 0;
  lastTheta2_joint = 0;
  lastTheta3_joint = 0;
}
// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleMove() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("theta1").toInt();
  targetTheta2_i = server.arg("theta2").toInt();
  targetTheta3_i = server.arg("theta3").toInt();
  lastGripper = server.arg("gripper").toInt();
  lastSpeed = server.arg("speed").toInt();
  lastAcceleration = server.arg("acceleration").toInt();
  Arm_mode = String(server.arg("mode"));
  Serial.println("------handleMove-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);
  Serial.println(lastGripper);
  Serial.println(lastSpeed);
  Serial.println(lastAcceleration);
  Serial.println(Arm_mode);
  Serial.println("-------------");
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  trajectoryPlanning(targetTheta1_i, targetTheta2_i, targetTheta3_i, 7);

  // ควบคุม Gripper
  gripperServo.write(lastGripper);

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/index");  // กลับไปที่ URL "/"
  server.send(303);                         // 303: See Other (Redirect to GET)
}


void movejoint() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("theta1").toInt();
  targetTheta2_i = server.arg("theta2").toInt();
  targetTheta3_i = server.arg("theta3").toInt();
  lastGripper = server.arg("gripper").toInt();
  lastSpeed = server.arg("speed").toInt();
  lastAcceleration = server.arg("acceleration").toInt();
  Arm_mode = String(server.arg("mode"));
  Serial.println("------handleMove-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);
  Serial.println(lastGripper);
  Serial.println(lastSpeed);
  Serial.println(lastAcceleration);
  Serial.println(Arm_mode);
  Serial.println("-------------");
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  Planning(targetTheta1_i, targetTheta2_i, targetTheta3_i, 7);

  // ควบคุม Gripper
  gripperServo.write(lastGripper);

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/joint");  // กลับไปที่ URL "/"
  server.send(303);                         // 303: See Other (Redirect to GET)
}

void handleOutput() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("io").toInt();
  state = server.arg("state");
  Serial.println("------Output-------");
  Serial.println(targetTheta1_i);
  Serial.println(state);

  Command cmd;
  cmd.type = "output";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleIf() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("io").toInt();
  state = server.arg("state");
  Serial.println("------if-------");
  Serial.println(targetTheta1_i);
  Serial.println(state);

  Command cmd;
  cmd.type = "if";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleGripper() {
  // รับค่าจากฟอร์ม
  lastGripper = server.arg("gripper").toInt();


  Serial.println("------Gripper-------");

  Serial.println(lastGripper);
  // ควบคุม Gripper
  //
  Command cmd;
  cmd.type = "Gripper";
  cmd.theta1 = lastGripper;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleDelay() {
  // รับค่าจากฟอร์ม
  delay_time = server.arg("time").toInt();



  Serial.println("------delay_time-------");
  Serial.println(delay_time);
  Command cmd;
  cmd.type = "delay";
  cmd.theta1 = delay_time;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ควบคุม Gripper

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}

// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleFor() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("BY").toInt();
  targetTheta2_i = server.arg("FROM").toInt();
  targetTheta3_i = server.arg("TO").toInt();

  Serial.println("------for-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);


  Command cmd;
  cmd.type = "for";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = targetTheta2_i;
  cmd.theta3 = targetTheta3_i;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleHome() {
  Serial.println("------home-------");

  Command cmd;
  cmd.type = "home";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleZero() {
  Serial.println("------zero-------");

  Command cmd;
  cmd.type = "zero";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void powerOffMotors() {
  stepper1.disableOutputs();
  stepper2.disableOutputs();
  stepper3.disableOutputs();
}
// ฟังก์ชันสำหรับบันทึกข้อความลงใน EEPROM
void writeStringToEEPROM(int addr, const String& data) {
  len = data.length();
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + len, 0);  // Null terminator
  EEPROM.commit();              // บันทึกข้อมูลลง EEPROM จริงๆ
}

// ฟังก์ชันสำหรับอ่านข้อความจาก EEPROM
String readStringFromEEPROM(int addr) {
  memset(data, 0, sizeof(data));

  len = 0;

  k = EEPROM.read(addr);
  Serial.println("-------------");
  while (k != 0 && len < 32) {
    data[len] = k;  // เก็บค่าที่อ่านได้ในตัวแปร data
    len++;
    k = EEPROM.read(addr + len);  // อ่านตัวอักษรถัดไปจาก EEPROM
    Serial.println(k);            // แสดงค่าที่อ่านจาก EEPROM เพื่อวิเคราะห์
  }
  Serial.println("-------------");
  data[len] = '\0';  // สิ้นสุด string ด้วย null terminator

  return String(data);  // แปลง char array เป็น String และส่งคืน
}

// ==================== API ENDPOINTS ====================

// API: Move robot arm to specific position
void handleApiMove() {
  if (server.method() != HTTP_POST) {
    sendApiError("Method not allowed. Use POST.", 405);
    return;
  }

  // Parse JSON or form data
  int x = server.arg("x").toInt();
  int y = server.arg("y").toInt();
  int z = server.arg("z").toInt();
  int speed = server.hasArg("speed") ? server.arg("speed").toInt() : lastSpeed;
  int acceleration = server.hasArg("acceleration") ? server.arg("acceleration").toInt() : lastAcceleration;

  if (x == 0 && y == 0 && z == 0 && !server.hasArg("x")) {
    sendApiError("Missing required parameters: x, y, z");
    return;
  }

  lastSpeed = speed;
  lastAcceleration = acceleration;
  setStepperSpeedAndAcceleration(speed, acceleration);
  trajectoryPlanning(x, y, z, 7);

  String responseData = "{";
  responseData += "\"x\":" + String(x) + ",";
  responseData += "\"y\":" + String(y) + ",";
  responseData += "\"z\":" + String(z) + ",";
  responseData += "\"speed\":" + String(speed) + ",";
  responseData += "\"acceleration\":" + String(acceleration);
  responseData += "}";

  sendApiSuccess("Robot arm movement command sent", responseData);
}

// API: Move robot joints to specific angles
void handleApiJoint() {
  if (server.method() != HTTP_POST) {
    sendApiError("Method not allowed. Use POST.", 405);
    return;
  }

  int theta1 = server.arg("theta1").toInt();
  int theta2 = server.arg("theta2").toInt();
  int theta3 = server.arg("theta3").toInt();
  int speed = server.hasArg("speed") ? server.arg("speed").toInt() : lastSpeed;
  int acceleration = server.hasArg("acceleration") ? server.arg("acceleration").toInt() : lastAcceleration;

  if (theta1 == 0 && theta2 == 0 && theta3 == 0 && !server.hasArg("theta1")) {
    sendApiError("Missing required parameters: theta1, theta2, theta3");
    return;
  }

  lastSpeed = speed;
  lastAcceleration = acceleration;
  setStepperSpeedAndAcceleration(speed, acceleration);
  Planning(theta1, theta2, theta3, 7);

  String responseData = "{";
  responseData += "\"theta1\":" + String(theta1) + ",";
  responseData += "\"theta2\":" + String(theta2) + ",";
  responseData += "\"theta3\":" + String(theta3) + ",";
  responseData += "\"speed\":" + String(speed) + ",";
  responseData += "\"acceleration\":" + String(acceleration);
  responseData += "}";

  sendApiSuccess("Robot joint movement command sent", responseData);
}

// API: Control gripper
void handleApiGripper() {
  if (server.method() != HTTP_POST) {
    sendApiError("Method not allowed. Use POST.", 405);
    return;
  }

  if (!server.hasArg("position")) {
    sendApiError("Missing required parameter: position");
    return;
  }

  int position = server.arg("position").toInt();
  if (position < 0 || position > 180) {
    sendApiError("Gripper position must be between 0 and 180");
    return;
  }

  Command cmd;
  cmd.type = "Gripper";
  cmd.theta1 = position;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  lastGripper = position;

  String responseData = "{\"position\":" + String(position) + "}";
  sendApiSuccess("Gripper position set", responseData);
}

// API: Get robot status
void handleApiStatus() {
  if (server.method() != HTTP_GET) {
    sendApiError("Method not allowed. Use GET.", 405);
    return;
  }

  String responseData = "{";
  responseData += "\"current_position\":{";
  responseData += "\"stepper1\":" + String(stepper1.currentPosition()) + ",";
  responseData += "\"stepper2\":" + String(stepper2.currentPosition()) + ",";
  responseData += "\"stepper3\":" + String(stepper3.currentPosition());
  responseData += "},";
  responseData += "\"last_coordinates\":{";
  responseData += "\"x\":" + String(lastTheta1) + ",";
  responseData += "\"y\":" + String(lastTheta2) + ",";
  responseData += "\"z\":" + String(lastTheta3);
  responseData += "},";
  responseData += "\"joint_angles\":{";
  responseData += "\"theta1\":" + String(lastTheta1_joint) + ",";
  responseData += "\"theta2\":" + String(lastTheta2_joint) + ",";
  responseData += "\"theta3\":" + String(lastTheta3_joint);
  responseData += "},";
  responseData += "\"gripper_position\":" + String(lastGripper) + ",";
  responseData += "\"speed\":" + String(lastSpeed) + ",";
  responseData += "\"acceleration\":" + String(lastAcceleration) + ",";
  responseData += "\"queue_size\":" + String(commandQueue.size()) + ",";
  responseData += "\"wifi_connected\":";
  responseData += (WiFi.status() == WL_CONNECTED ? "true" : "false");
  responseData += ",";
  responseData += "\"ip_address\":\"" + WiFi.localIP().toString() + "\",";
  responseData += "\"arm_mode\":\"" + Arm_mode + "\"";
  responseData += "}";

  sendApiSuccess("Robot status retrieved", responseData);
}

// API: Execute G-code
void handleApiGcode() {
  if (server.method() != HTTP_POST) {
    sendApiError("Method not allowed. Use POST.", 405);
    return;
  }

  if (!server.hasArg("gcode")) {
    sendApiError("Missing required parameter: gcode");
    return;
  }

  String gcode = server.arg("gcode");
  int commandsProcessed = 0;
  String processedCommands = "[";

  int lineStart = 0;
  while (lineStart < gcode.length()) {
    int lineEnd = gcode.indexOf('\n', lineStart);
    if (lineEnd == -1) lineEnd = gcode.length();
    String line = gcode.substring(lineStart, lineEnd);
    line.trim();
    
    if (line.length() > 0) {
      String type;
      int xVal = 0, yVal = 0, zVal = 0, delayVal = 0;
      parseGcodeLine(line, type, xVal, yVal, zVal, delayVal);

      if (commandsProcessed > 0) processedCommands += ",";
      processedCommands += "{\"line\":\"" + line + "\",\"type\":\"" + type + "\"";

      if (type == "XYZ") {
        trajectoryPlanning(xVal, yVal, zVal, delayVal);
        processedCommands += ",\"x\":" + String(xVal) + ",\"y\":" + String(yVal) + ",\"z\":" + String(zVal);
      } else if (type == "GPON" || type == "GPOFF") {
        Command cmd;
        cmd.type = "Gripper";
        cmd.theta1 = (type == "GPON") ? 0 : 180;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        processedCommands += ",\"gripper_position\":" + String(cmd.theta1);
      } else if (type == "DELAY") {
        Command cmd;
        cmd.type = "delay";
        cmd.theta1 = delayVal;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        processedCommands += ",\"delay\":" + String(delayVal);
      }
      processedCommands += "}";
      commandsProcessed++;
    }
    lineStart = lineEnd + 1;
  }
  processedCommands += "]";

  String responseData = "{\"commands_processed\":" + String(commandsProcessed) + ",\"commands\":" + processedCommands + "}";
  sendApiSuccess("G-code executed", responseData);
}

// API: Home robot
void handleApiHome() {
  if (server.method() != HTTP_POST) {
    sendApiError("Method not allowed. Use POST.", 405);
    return;
  }

  Command cmd;
  cmd.type = "home";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  sendApiSuccess("Robot homing command sent");
}

// API: Zero robot (calibrate)
void handleApiZero() {
  if (server.method() != HTTP_POST) {
    sendApiError("Method not allowed. Use POST.", 405);
    return;
  }

  Command cmd;
  cmd.type = "zero";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  sendApiSuccess("Robot calibration command sent");
}

// API: Handle OPTIONS requests for CORS
void handleApiOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  server.send(200);
}

// API Documentation Page
void handleApiDocs() {
  String html = generateApiDocPage();
  server.send(200, "text/html", html);
}
void setup() {
  Serial.begin(115200);

  EEPROM.begin(500);

  // ตรวจสอบว่าเคยบันทึก SSID และ Password หรือไม่

  // ตั้งค่า Limit Switch เป็น INPUT_PULLUP
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  // เชื่อมต่อ Wi-Fi
  // เชื่อมต่อ Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }

  // // แสดง IP Address บน Serial Monitor
  // Serial.println("Connected to WiFi");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());  // แสดง IP Address ของ ESP32
  // writeStringToEEPROM(0, "0");
  // writeStringToEEPROM(32 ,"0");
  savedSSID = readStringFromEEPROM(0);
  savedPassword = readStringFromEEPROM(32);
  ip = readStringFromEEPROM(64);
  gateway = readStringFromEEPROM(96);
  subnet = readStringFromEEPROM(128);

  Serial.println(savedSSID);
  Serial.println(savedPassword);

  if (savedSSID.length() < 2) {
    Serial.println("Starting in AP mode...");
    isFirstTime = true;
    WiFi.softAP(AP_name, "12345678");

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  } else {
    // ตรวจสอบและตั้งค่า IP แบบคงที่จากข้อมูลที่บันทึก
    if (ip.length() > 0 && gateway.length() > 0 && subnet.length() > 0) {
      IPAddress local_IP, local_gateway, local_subnet;
      local_IP.fromString(ip);
      local_gateway.fromString(gateway);
      local_subnet.fromString(subnet);

      if (!WiFi.config(local_IP, local_gateway, local_subnet)) {
        Serial.println("STA Failed to configure");
      }
    }

    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
      wifi_connecy_count++;
      if (wifi_connecy_count > 20) {
        writeStringToEEPROM(0, "0");
        writeStringToEEPROM(32, "0");
        ESP.restart();
      }
    }

    Serial.println("");
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  // เริ่มต้นการตั้งค่าเซอร์โวและมอเตอร์
  gripperServo.attach(13);
  gripperServo.write(180);
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  // เพิ่มมอเตอร์ทั้งหมดใน MultiStepper สำหรับการควบคุมแบบประสานงาน
  multiStepper.addStepper(stepper1);
  multiStepper.addStepper(stepper2);
  multiStepper.addStepper(stepper3);

  //setZero();
  //setHomePosition();
  server.enableCORS();
  // เริ่มต้น Web Server
  server.on("/", handleMain);          // หน้าเว็บหลัก
  server.on("/index", handleRoot);     // หน้าเว็บหลัก
  server.on("/joint", handlejoint);    // หน้าเว็บหลัก
  server.on("/move", handleMove);      // รับคำสั่งการเคลื่อนที่
  server.on("/movejoint", movejoint);  // รับคำสั่งการเคลื่อนที่
  server.on("/sethomevalue", setHomeValue);
  server.on("/gohomevalue", goHomeValue);
  server.on("/gcode", HTTP_GET, handleGcodePage);     // หน้า input G-code
  server.on("/gcode", HTTP_POST, handleGcodeSubmit);  // รับข้อมูล G-code
  server.on("/setup", handleSetup);                   // หน้าเว็บหลัก
  server.on("/setuparm", handleSetupPage);            // หน้าเว็บหลัก
  server.on("/save-setup", handleSaveWiFi);
  server.on("/save-param", handleSaveSetupArm);
  server.on("/controlGripper", handleGripper);
  server.on("/delay", handleDelay);
  server.on("/for", handleFor);
  server.on("/home", handleHome);
  server.on("/zero", handleZero);
  server.on("/ota", handleOta);  //
  server.on("/output", handleOutput);
  server.on("/if", handleIf);

  // ==================== API ROUTES ====================
  server.on("/api/move", HTTP_POST, handleApiMove);
  server.on("/api/joint", HTTP_POST, handleApiJoint);
  server.on("/api/gripper", HTTP_POST, handleApiGripper);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/gcode", HTTP_POST, handleApiGcode);
  server.on("/api/home", HTTP_POST, handleApiHome);
  server.on("/api/zero", HTTP_POST, handleApiZero);
  server.on("/api/docs", HTTP_GET, handleApiDocs);
  
  // Handle CORS preflight requests
  server.on("/api/move", HTTP_OPTIONS, handleApiOptions);
  server.on("/api/joint", HTTP_OPTIONS, handleApiOptions);
  server.on("/api/gripper", HTTP_OPTIONS, handleApiOptions);
  server.on("/api/status", HTTP_OPTIONS, handleApiOptions);
  server.on("/api/gcode", HTTP_OPTIONS, handleApiOptions);
  server.on("/api/home", HTTP_OPTIONS, handleApiOptions);
  server.on("/api/zero", HTTP_OPTIONS, handleApiOptions);
  /*handling uploading firmware file */
  server.on(
    "/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    },
    []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {  //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {  //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
  server.begin();
  Serial.println("Web server started");
  // ตั้งค่า Timer ให้เรียกใช้ `onTimer` ทุกๆ 1 ms
  timer = timerBegin(0, 80, true);  // Timer 0, Prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);  // เรียกทุกๆ 1000 ticks = 1 ms
  timerAlarmEnable(timer);             // เปิดใช้งาน Timer
}

void loop() {
  serial();
  if (!commandQueue.empty()) {
    Command cmd = commandQueue.front();
    if (!commandQueue.empty()) {

      if (cmd.type == "move") {
        executeCommand(cmd);
      } else if (cmd.type == "delay") {
        delay(cmd.theta1);
      } else if (cmd.type == "Gripper") {
        gripperServo.write(cmd.theta1);
      } else if (cmd.type == "home") {
        setHomeArm();
      } else if (cmd.type == "zero") {
        setZero();
      } else if (cmd.type == "output") {

      } else {
        // Handle other cases if needed
      }
    }
    commandQueue.pop();
  }



  // จัดการคำร้องขอ HTTP
  server.handleClient();
}
