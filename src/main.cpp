#include <Arduino.h>
#include "BluetoothSerial.h"
#include "driveMotor.h"
#include <string.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include "PID.h"
#include "MPU6050Helper.h"
#include <Wire.h>

#define IN1 26
#define IN2 27
#define ENA 25
#define IN1_2 12
#define IN2_2 13
#define ENB 14
#define btn_setup 15
#define btn_submit 4
#define btn_increase 16
#define btn_decrease 17
#define led_demo 2

BluetoothSerial SerialBT;
DriveMotor Motor1, Motor2;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences preferences;
MPU6050Helper mpuHelper;

int speed_pwm;
int value_max_speed = 100;
float value_max_kpid =  150;
int state = 0;
int state_old = state;
int begin_signal = 0;
String sendCMD = "ESP32 to APP";
char str_print1[25];
char str_print2[25];
int number_ID = 4;
char *numberAdd[4] = {"Speed","Kp","Kd","Ki"};
float speed_crt, kp_crt, ki_crt, kd_crt;
boolean signal_change_state = false;
String recvDataBLT;
int state_config_mpu;
float ax, ay, az, gx, gy, gz, temp;
sensors_event_t a,g,t;
int angle;
unsigned long lastTimeMPU, crtState1, crtState2, crtState3, crtState4;
float dt;
int state_run1, state_run2 = 0;
String old_data_rcv;

void writeDataToEEPROM(int address, const char* data);
String readDataToEEPROM(int address);
boolean checkAddressEEPROM(int address);
void clearAdressEEPROM(int address);

void writeDataToEEPROM(char *nameKey, float data){
  preferences.putFloat(nameKey, data);
}

float readDataToEEPROM(char *nameKey){
  float value = preferences.getFloat(nameKey, -1);
  return value;
}



void initialize_parameters_eeprom(){
  int number_full = 0;
  // Khởi tạo tham số ban đầu khi cài đặt
  for (int i = 0; i<number_ID; i++){
    float value = readDataToEEPROM(numberAdd[i]);
    if (value != -1){
      number_full ++;
    }
  }
  
  if (number_full == 0){
    for (int i = 0; i<number_ID; i++){
      writeDataToEEPROM(numberAdd[i], 0);
    }
  }
    
}

void clearAdressEEPROM(char *nameKey){
 
  preferences.putString(nameKey, "");

}


void config_lcd(){
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void configMotor1(){
    Motor1.in1_pin = IN1;
    Motor1.in2_pin = IN2;
    Motor1.en_pin = ENA;

}

void configMotor2(){
    Motor2.in1_pin = IN1_2;
    Motor2.in2_pin = IN2_2;
    Motor2.en_pin = ENB;

}



boolean find_string(String stringChild, String stringParent){
    int nSizeSC = stringChild.length();
    int nTrue = 0;
    for (int i = 0; i<nSizeSC;i++){
        if (stringChild[i] == stringParent[i]){
            nTrue ++;
        }

    }
    if (nTrue == nSizeSC){
        return true;
    }
    else{
        return false;
    }
}

String get_last_str_receive(String str_value){
    int last_index_ = str_value.lastIndexOf("_");
    String last_receive = str_value.substring(last_index_+1);
    return last_receive;
}

int get_last_speed(String str_value){
    int last_speed_index = str_value.lastIndexOf("-");
    int value_last_speed = str_value.substring(last_speed_index+1).toInt();
    return value_last_speed;
}

void right_site_car(DriveMotor M1, DriveMotor M2, int speed){ // quạch phải
    M1.stop_motor();
    M2.run_forward(speed);
    
}

void left_site_car(DriveMotor M1, DriveMotor M2, int status_run, int speed){ // quạch trái
    M2.stop_motor();
    M1.run_forward(speed);

}

void stop_car(DriveMotor M1, DriveMotor M2){ // quạch trái
    M2.stop_motor();
    M1.stop_motor();

}


float btn_increase_clicked(char *name_type, float value_crt, int val_max, float step)
{
  if (digitalRead(btn_increase) == 0){
    while(digitalRead(btn_increase) == 0);
    value_crt = value_crt + step;
    if (value_crt > val_max){
      value_crt = val_max;
    }
    sprintf(str_print1, "Setup %s", name_type);
    lcd.setCursor(0,0);
    lcd.print(str_print1);
    sprintf(str_print2, "%s:%.1f", name_type, value_crt);
    lcd.setCursor(0, 1);  
    lcd.print(str_print2);
    Serial.println("Tang");
    writeDataToEEPROM(name_type, value_crt);
  }
  return value_crt;
}


float btn_decrease_clicked(char *name_type, float value_crt, int val_min, float step)
{
  if (digitalRead(btn_decrease) == 0){
    while(digitalRead(btn_decrease) == 0);
    value_crt = value_crt - step;
    if (value_crt < val_min){
      value_crt = val_min;
    }
    sprintf(str_print1, "Setup %s", name_type);
    lcd.setCursor(0,0);
    lcd.print(str_print1);
    sprintf(str_print2, "%s:%.1f", name_type, value_crt);
    lcd.setCursor(0, 1);  
    lcd.print(str_print2);
    Serial.println("Giam");
    writeDataToEEPROM(name_type, value_crt);
  }
  return value_crt;
}



void change_state(){
  if (digitalRead(btn_setup) == 0){
    while(digitalRead(btn_setup) == 0);
    state ++;
    begin_signal = 0;
  }
}

float change_value(char *name_value, int index_id){
  stop_car(Motor1, Motor2);
  // Doc  du lieu tu eeprom 
  lcd.clear();
  sprintf(str_print1, "SetUp %s", name_value);
  lcd.setCursor(0, 0);
  lcd.print(str_print1);
  float value_crt = readDataToEEPROM(numberAdd[index_id]);
  lcd.setCursor(0, 1);
  sprintf(str_print2, "%s:%.1f", name_value, value_crt);
  lcd.print(str_print2);
  return value_crt;
}

String receive_data_BLT(){
  String rcvData = SerialBT.readString();

  return rcvData;
}
float getAngle(float ax, float ay, float az, float gx, float gy, float gz, float dt, float angle, float para_range) {
  
  static float gyroRate = 0.0;
  static float accelAngle = 0.0;

  // Tính góc từ gia tốc
  accelAngle = atan2(ay, az) * 180 / PI;

  // Tích hợp tốc độ góc để tính góc từ con quay
  gyroRate = gx / para_range; // 131.0 là giá trị chia của con quay (tùy theo config)
  angle = 0.9 * (angle + gyroRate * dt) + 0.1 * accelAngle;

  return angle;
}

float get_value_speed_btl(String data_speed){
  String number = "";
  for (int i = 0; i < data_speed.length(); i++){
    if (isdigit(data_speed[i])){
      number += data_speed[i];

    }
  }
  float value = number.toFloat();
  return value;
}

float get_value_pid_blt(String data_k){
  String number = "";
  for (int i = 0; i< data_k.length(); i++){
    if (isdigit(data_k[i])){
      number += data_k[i];
    }
    if (data_k[i] == '.'){
      number += data_k[i];
    }
  }
  float value = number.toFloat();
  return value;
}


void process_cmd_blt(String data_cmd){
  if ((data_cmd.lastIndexOf("UP-RIGHT\n") != -1 || data_cmd.lastIndexOf("RIGHT-UP\n") != -1) && data_cmd.length() == 9) {
    Serial.println("Di thang cheo phai");
  }
  if ((data_cmd.lastIndexOf("UP-LEFT\n") != -1 || data_cmd.lastIndexOf("LEFT-UP\n") != -1) && data_cmd.length() == 8) {
      Serial.println("Di thang cheo trai");
  }
  if ((data_cmd.lastIndexOf("DOWN-RIGHT\n") != -1 || data_cmd.lastIndexOf("RIGHT-DOWN\n") != -1) && data_cmd.length() == 11) {
      Serial.println("Di xuong cheo phai");
  }
  if ((data_cmd.lastIndexOf("DOWN-LEFT\n") != -1 || data_cmd.lastIndexOf("LEFT-DOWN\n") != -1) && data_cmd.length() == 10) {
      Serial.println("Di xuong cheo trai");
  }
  if (data_cmd.lastIndexOf("LEFT\n") != -1 && data_cmd.length() == 5) {
      Serial.println("Quach trai");
  }
  if (data_cmd.lastIndexOf("RIGHT\n") != -1 && data_cmd.length() == 6) {
      Serial.println("Quach phai");
  }
  if (data_cmd.lastIndexOf("DOWN\n") != -1 && data_cmd.length() == 5) {
      Serial.println("Di xuong");
  }
  if (data_cmd.lastIndexOf("UP\n") != -1 && data_cmd.length() == 3) {
      Serial.println("Di thang");
  }

  

}

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(IN1_2, OUTPUT);
    pinMode(IN2_2, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(btn_setup, INPUT);
    pinMode(btn_decrease, INPUT);
    pinMode(btn_increase, INPUT);
    pinMode(btn_submit, INPUT);

    pinMode(led_demo, OUTPUT);


    Serial.begin(9600);
    SerialBT.begin("ESP32_BalanceCar"); // Tên thiết bị Bluetooth
    Serial.println("Bluetooth Started!"); 
    configMotor1();
    configMotor2();
    config_lcd();
    preferences.begin("my-app", false);
    initialize_parameters_eeprom();
    if (!mpuHelper.begin()) {
        Serial.println("MPU6050 Connect Fail!");
        while (1);
    }
    Serial.println("MPU6050 ready");
    
    Wire.begin();
}





void loop() {
  if (state == 0){
    recvDataBLT = receive_data_BLT();
    if (recvDataBLT.length() > 1){
      old_data_rcv = recvDataBLT;
    }
    if (begin_signal == 0){
    
      speed_crt = readDataToEEPROM(numberAdd[0]);
      kp_crt = readDataToEEPROM(numberAdd[1]);
      kd_crt = readDataToEEPROM(numberAdd[2]);
      ki_crt = readDataToEEPROM(numberAdd[3]);
      
      lcd.clear();
      sprintf(str_print1, "SP:%.1f Kp:%.1f     ", speed_crt, kp_crt);
      sprintf(str_print2, "Kd:%.1f Ki:%.1f     ", kd_crt, ki_crt);
      lcd.setCursor(0, 0);
      lcd.print(str_print1);
      lcd.setCursor(0,1);
      lcd.print(str_print2);
      begin_signal = 1;
      
    }
    
    // MPU
    unsigned long currentTimeMPU = millis(); // Thời gian hiện tại (ms)
    dt = (currentTimeMPU - lastTimeMPU); // Chuyển sang giây
    if (dt >= 1000){
      lastTimeMPU = currentTimeMPU; // Cập nhật thời gian lần trước
      dt = dt/1000;
      mpuHelper.getAllMPU(ax, ay, az, gx, gy, gz, temp);
      angle = (int)getAngle(ax, ay, az, gx, gy, gz, dt, angle, 65.5);
      Serial.println("Angle o");
      Serial.println(angle);
      PID pid(kp_crt, ki_crt, kd_crt);
      pid.setSetpoint(0.00);
      int pid_output = (int) pid.compute(angle, dt); // Tính toán tín hiệu điều khiển
      Serial.println("PID");
      Serial.println(pid_output);
        // Giới hạn giá trị đầu ra
      
      if (pid_output > 0) {
          if (state_run1 == 0){
            Motor1.run_forward(130);  // Động cơ quay tiến với tốc độ `pid_output`.
            Motor2.run_forward(130);
            state_run1 = 1;
            state_run2 = 0;
            delay(200);
          }
          Motor1.run_forward(speed_crt);  // Động cơ quay tiến với tốc độ `pid_output`.
          Motor2.run_forward(speed_crt);
      } else {

          if (state_run2 == 0){
            Motor1.run_backward(130);  // Động cơ quay tiến với tốc độ `pid_output`.
            Motor2.run_backward(130);
            state_run2 = 1;
            state_run1 = 0;
            delay(200);
          }
          

          Motor1.run_backward(speed_crt); // Động cơ quay lùi với tốc độ `-pid_output`.
          Motor2.run_backward(speed_crt);
      }
      
      
      
    }
    process_cmd_blt(old_data_rcv);

    change_state();
    
  }
  else if (state == 1){
    if (begin_signal == 0){
      stop_car(Motor1, Motor2);
      // Doc  du lieu tu eeprom 
      lcd.clear();
      sprintf(str_print1, "SetUp Speed");
      lcd.setCursor(0, 0);
      lcd.print(str_print1);
      speed_crt = readDataToEEPROM(numberAdd[0]);
      lcd.setCursor(0, 1);
      sprintf(str_print2, "Speed:%.1f", speed_crt);
      lcd.print(str_print2);

      delay(100);
      begin_signal = 1;
    }
    unsigned long currentTimeMPU = millis(); // Thời gian hiện tại (ms)
    dt = (currentTimeMPU - lastTimeMPU); // Chuyển sang giây
    if (dt >= 200){\
      lastTimeMPU = currentTimeMPU;
      change_state();

      speed_crt = btn_increase_clicked(numberAdd[0], speed_crt, value_max_speed, 1);
      speed_crt = btn_decrease_clicked(numberAdd[0], speed_crt, 0, 1);
    }

    
  }
  else if (state == 2){
    if (begin_signal == 0){
      kp_crt = change_value("Kp", 1);
      delay(100);
      begin_signal = 1;
    }
    unsigned long currentTimeMPU = millis(); // Thời gian hiện tại (ms)
    dt = (currentTimeMPU - lastTimeMPU); // Chuyển sang giây
    if (dt >= 200){
      lastTimeMPU = currentTimeMPU;
      change_state();
      kp_crt = btn_increase_clicked(numberAdd[1], kp_crt, value_max_kpid, 0.1);
      kp_crt = btn_decrease_clicked(numberAdd[1], kp_crt, 0, 0.1);
    }
  }

  else if (state == 3){
    if (begin_signal == 0){
      kp_crt = change_value("Kd", 2);
      delay(100);
      begin_signal = 1;
    }
    unsigned long currentTimeMPU = millis(); // Thời gian hiện tại (ms)
    dt = (currentTimeMPU - lastTimeMPU); // Chuyển sang giây
    if (dt >= 200){
      lastTimeMPU = currentTimeMPU;
      change_state();
      kd_crt = btn_increase_clicked(numberAdd[2], kd_crt, value_max_kpid, 0.1);
      kd_crt = btn_decrease_clicked(numberAdd[2], kd_crt, 0, 0.1);
    }
  }

  else if (state == 4){
    if (begin_signal == 0){
      ki_crt = change_value("Ki", 3);
      delay(100);
      begin_signal = 1;
    }
    unsigned long currentTimeMPU = millis(); // Thời gian hiện tại (ms)
    dt = (currentTimeMPU - lastTimeMPU); // Chuyển sang giây
    if (dt >= 200){
      lastTimeMPU = currentTimeMPU;
      change_state();
      ki_crt = btn_increase_clicked(numberAdd[3], ki_crt, value_max_kpid, 0.1);
      ki_crt = btn_decrease_clicked(numberAdd[3], ki_crt, 0, 0.1);
    }
  }

  else {
    state = 0 ;
  }

    
}
