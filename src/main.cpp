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
String final_data_rcv, final_old_data_rcv;
float setpoint = 0.00;
int pid_output;

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
    if (data_k[i] == ','){
      number += '.';
    }
  }
  float value = number.toFloat();
  return value;
}

// Khi gửi bị nhiều lệnh 1 lúc
boolean check_cmd(String str_data){
  int arr_n[10];
  int n_index = 0;
  int leng_arr = 0;
  // Lấy vị trí ký tự xuống dòng '\n'
  for (int i = 0; i < str_data.length(); i++) {
    if (str_data[i] == '\n') {
      Serial.println(i);
      arr_n[n_index] = i;
      n_index++;
    }
  }
  // Nếu có ít nhất hai dấu '\n' thì mới thực hiện lấy chuỗi con
  if (n_index > 1 && n_index<9) {
    return true;
  }
  else{
    return false;
  }
}
String process_long_cmd(String str_data) {
  int arr_n[10];
  int n_index = 0;
  int leng_arr = 0;
  // Lấy vị trí ký tự xuống dòng '\n'
  for (int i = 0; i < str_data.length(); i++) {
    if (str_data[i] == '\n') {
      Serial.println(i);
      arr_n[n_index] = i;
      n_index++;
    }
  }
  Serial.println(n_index);
  // Nếu có ít nhất hai dấu '\n' thì mới thực hiện lấy chuỗi con
  if (n_index > 1 && n_index<9) {
    String new_data="";
    Serial.println("Cắt chuỗi");
    // Lấy chuỗi từ vị trí sau dấu '\n' thứ n_index-1 đến vị trí dấu '\n' thứ n_index
    int start = arr_n[n_index -2]+1;
    int end = arr_n[n_index-1];
    for (int i = start; i<= end; i++){
      new_data += str_data[i];
    }
    return new_data;
  } else {
    // Trả về chuỗi rỗng nếu không đủ dấu '\n'
    return "";
  }
}


String process_cmd_blt(String data_cmd, int pid_output){
  Serial.println(data_cmd);
  
  if ((data_cmd.lastIndexOf("UP-RIGHT\n") != -1 || data_cmd.lastIndexOf("RIGHT-UP\n") != -1) && data_cmd.length() == 9) {
    Serial.println("Di thang cheo phai");
    setpoint = 1;
    // Motor1 = speedcrt, Mootor2 = 50%speedcrt
    Motor1.run_forward(speed_crt+pid_output);
    Motor2.run_forward(speed_crt);
    return data_cmd;
  }
  if ((data_cmd.lastIndexOf("UP-LEFT\n") != -1 || data_cmd.lastIndexOf("LEFT-UP\n") != -1) && data_cmd.length() == 8) {
      Serial.println("Di thang cheo trai");
      setpoint = 1;

      Motor1.run_forward(speed_crt);
      Motor2.run_forward(speed_crt+pid_output);
      return data_cmd;
  }
  if ((data_cmd.lastIndexOf("DOWN-RIGHT\n") != -1 || data_cmd.lastIndexOf("RIGHT-DOWN\n") != -1) && data_cmd.length() == 11) {
      Serial.println("Di xuong cheo phai");
      setpoint = -1;

      Motor1.run_backward(speed_crt+pid_output);
      Motor2.run_backward(speed_crt);
      return data_cmd;
  }
  if ((data_cmd.lastIndexOf("DOWN-LEFT\n") != -1 || data_cmd.lastIndexOf("LEFT-DOWN\n") != -1) && data_cmd.length() == 10) {
      Serial.println("Di xuong cheo trai");
      setpoint = -1;

      Motor1.run_backward(speed_crt);
      Motor2.run_backward(speed_crt+pid_output);
      return data_cmd;
  }
  if (data_cmd.lastIndexOf("LEFT\n") != -1 && data_cmd.length() == 5) {
      Serial.println("Quach trai");
      setpoint = 1;

      Motor1.run_forward(speed_crt+pid_output);
      Motor2.run_forward(40);
      return data_cmd;
  }
  if (data_cmd.lastIndexOf("RIGHT\n") != -1 && data_cmd.length() == 6) {
      Serial.println("Quach phai");
      setpoint = 1;

      Motor1.run_forward(40);
      Motor2.run_forward(speed_crt+pid_output);
      return data_cmd;
  }
  if (data_cmd.lastIndexOf("DOWN\n") != -1 && data_cmd.length() == 5) {
      Serial.println("Di xuong");
      Motor1.run_backward(speed_crt+pid_output);
      Motor2.run_backward(speed_crt+pid_output);
      return data_cmd;
  }
  if (data_cmd.lastIndexOf("UP\n") != -1 && data_cmd.length() == 3) {
      Serial.println("Di thang");
      Motor1.run_forward(speed_crt+pid_output);
      Motor2.run_forward(speed_crt+pid_output);
      return data_cmd;
  }

  if(data_cmd.indexOf("Kp") != -1 && data_cmd.length() > 2 && data_cmd.indexOf("Ki") == -1 && data_cmd.indexOf("Kd") == -1){
    Serial.println("Cap nhat Kp");
    begin_signal = 0;
    // lay gia tri speed
    float kp_new = get_value_pid_blt(data_cmd);
    Serial.println(kp_new);
    writeDataToEEPROM(numberAdd[1], kp_new);
    return "";
  }

  else if(data_cmd.indexOf("Kd") != -1 && data_cmd.length() > 2 && data_cmd.indexOf("Ki") == -1 && data_cmd.indexOf("Kp") == -1){
    Serial.println("Cap nhat Kd");
    begin_signal = 0;
    // lay gia tri speed
    float kd_new = get_value_pid_blt(data_cmd);
    Serial.println(kd_new);
    writeDataToEEPROM(numberAdd[2], kd_new);
    return "";
  }

  else if(data_cmd.indexOf("Ki") != -1 && data_cmd.length() > 2 && data_cmd.indexOf("Kd") == -1 && data_cmd.indexOf("Kp") == -1){
    Serial.println("Cap nhat Ki");
    begin_signal = 0;
    // lay gia tri speed
    float ki_new = get_value_pid_blt(data_cmd);
    Serial.println(ki_new);
    writeDataToEEPROM(numberAdd[3], ki_new);
    return "";
  }
  else if(data_cmd.indexOf("Speed") != -1 && data_cmd.length() > 2 && data_cmd.indexOf("Kp") == -1 && data_cmd.indexOf("Kd") == -1 && data_cmd.indexOf("Ki") == -1){
    Serial.println("Cap nhat Speed");
    begin_signal = 0;
    // lay gia tri speed
    float sp_new = get_value_speed_btl(data_cmd);
    Serial.println(sp_new);
    writeDataToEEPROM(numberAdd[0], sp_new);
    return "";
  }
  else{
    Serial.println("Xe can bang tai cho");
    setpoint = 0;
    if (pid_output > 0) {
          
          Motor1.run_forward(speed_crt+pid_output);  // Động cơ quay tiến với tốc độ `pid_output`.
          Motor2.run_forward(speed_crt+pid_output);
      } else {

      Motor1.run_backward(speed_crt+pid_output); // Động cơ quay lùi với tốc độ `-pid_output`.
      Motor2.run_backward(speed_crt+pid_output);
      }
  
    return "";
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
      if ( SerialBT.connected()){
        char vl_mpu[100];
        sprintf(vl_mpu, "ax:%.2f ay:%.2f az:%.2f gx:%.2f gy:%.2f gz:%.2f SP:%.1f Kp:%.1f Kd:%.1f Ki:%.1f", ax, ay, az, gx, gy, gz, speed_crt, kp_crt,  kd_crt, ki_crt);
        SerialBT.println(vl_mpu);
        Serial.println("Đã gửi");
      }
      
      angle = (int)getAngle(ax, ay, az, gx, gy, gz, dt, angle, 65.5);
      Serial.println("Angle o");
      Serial.println(angle);
      PID pid(kp_crt, ki_crt, kd_crt);
      pid.setSetpoint(setpoint);
      pid_output = (int) pid.compute(angle, dt); // Tính toán tín hiệu điều khiển
      pid_output = pid.constrainOutput(-100, 100);
      Serial.println("Setpoint: ");
      Serial.println(setpoint);
      Serial.println("PID");
      Serial.println(pid_output);
        // Giới hạn giá trị đầu ra
      if (pid_output > 0) {
          
          Motor1.run_forward(speed_crt+pid_output);  // Động cơ quay tiến với tốc độ `pid_output`.
          Motor2.run_forward(speed_crt+pid_output);
      } else {
      pid_output = -pid_output;
      Motor1.run_backward(speed_crt+pid_output); // Động cơ quay lùi với tốc độ `-pid_output`.
      Motor2.run_backward(speed_crt+pid_output);
      }
      recvDataBLT = receive_data_BLT();
      if (recvDataBLT.length() > 2){
        final_data_rcv = recvDataBLT;
        
      }
      if (check_cmd(final_data_rcv) == true){
        final_data_rcv = process_long_cmd(final_data_rcv);
        Serial.println("New: ");
        Serial.println(final_data_rcv);
      }
      if (final_data_rcv != final_old_data_rcv){
        final_old_data_rcv = final_data_rcv;
        final_data_rcv = process_cmd_blt(final_data_rcv, pid_output);
      }
      Serial.print("Che do: ");
      Serial.println(final_data_rcv);

    }
    
    
    

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
  delay(50);

    
}
