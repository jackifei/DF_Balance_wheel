#include "Arduino.h"
#include <MPU6050_tockn.h> //磁罗盘库
#include <Wire.h>          //I2C库
#include "SimpleFOC.h"     //电机库
#include <WiFi.h>
#include <WebServer.h>
// 左轮
// 自定义第一路 I2C 引脚
#define M0_I2C_SDA 39 // 通讯
#define M0_I2C_SCL 40 // 时钟
// 右轮

#define M1_I2C_SDA 41                // 通讯
#define M1_I2C_SCL 42                // 时钟
#define MOVING_AVERAGE_WINDOW_SIZE 5 // 定义移动平均滤波的窗口大小（例如，取前 n 个数据点的平均）

// 添加LED引脚定义，用于显示状态
const int ledPin1 = 9;  // 0.5HZ
const int ledPin2 = 10; // 0.2HZ
// 参数设置
float Balance_Angle_raw = 0.54;                            // 机械平衡角度
const int leftMotorOffset = 0.06, rightMotorOffset = 0.07; // 左右轮的启动扭矩值，pm到一定电压马达才开的转动.
float ENERGY = 3;                                          // 前进后退倾角，控制进后退速度
float kp = 8.8, ki = 0.19, kd = 0.29;                      // 根据调试设置kp ki kd的默认值，kp:8.8   ki:0.19   kd:0.29(平衡小车)
float turn_kp = 0.1;                                       // 转向kp值
float Keep_Angle, bias, integrate;                         // 保持角度，角度偏差，偏差积分变量
float AngleX, GyroX, GyroZ;                                // mpu6050输出的角度值为浮点数，两位有效小数
float vertical_PWM, turn_PWM, PWM, L_PWM, R_PWM;           // 各种PWM计算值
float turn_spd = 0;                                        // 转向Z角速度值，初始值为0
float turn_ENERGY = 300;                                   // 转向Z角速度增加值值
float gyroZBuffer[MOVING_AVERAGE_WINDOW_SIZE];             // 全局数组用于存储最近的 n 个角速度值
int bufferIndex = 0;

// ====== Wi-Fi 配置 ======
const char *ssid = "BalanceCar_AP"; // 自定义AP名称
const char *password = "12345678";  // 自定义密码（至少8位）

WebServer wifiServer(80); // 监听 8080 端口

// 左轮
MagneticSensorI2C sensor_0 = MagneticSensorI2C(AS5600_I2C); // 初始化AS5600传感器0
TwoWire I2C_0 = TwoWire(0);
// 右轮
MagneticSensorI2C sensor_1 = MagneticSensorI2C(AS5600_I2C); // 初始化AS5600传感器1
TwoWire I2C_1 = TwoWire(1);
// 接入陀螺仪传感器
MPU6050 mpu6050(I2C_0); // Wire只有两个接口，但I2C可以走总线的方式，所以将MPU6050和AS5600接到同一组SDA、SCL线上，是完全可以的
// 定义左轮子
BLDCMotor motor_0 = BLDCMotor(7);                      // 2804电机的极对数为7
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(4, 5, 6, 47); // PWM引脚为25，32，33，使能引脚为12，foc板与esp板接线要对应
// 定义右轮子
BLDCMotor motor_1 = BLDCMotor(7);                         // 2804电机的极对数为7
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(15, 16, 17, 48); // PWM引脚为25，32，33，使能引脚为12，foc板与esp板接线要对应

float target_angle_1 = 0; // 定义角度变量，也可以是扭矩，也可以是速度
float target_angle_2 = 0; // 定义角度变量，也可以是扭矩，也可以是速度

// 实例化
Commander command = Commander(Serial); // 使用串口发送命令
void doTarget1(char *cmd) { command.scalar(&target_angle_1, cmd); }
void doTarget2(char *cmd) { command.scalar(&target_angle_2, cmd); }
// 用于 LED 闪烁的变量
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
const long interval1 = 500; // 500ms 切换一次，实现 1Hz 闪烁（亮0.5s + 灭0.5s）
const long interval2 = 250; // 500ms 切换一次，实现 1Hz 闪烁（亮0.5s + 灭0.5s）

char flag = 's'; // 控制左转右转的标签

void io_init()
{
  // 初始化LED引脚为输出模式
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin1, LOW);
}
// 传感器初始化

void sensor_init()
{
  I2C_0.begin(M0_I2C_SDA, M0_I2C_SCL, 100000); // 定义传感器0接线引脚
  sensor_0.init(&I2C_0);
  I2C_1.begin(M1_I2C_SDA, M1_I2C_SCL, 100000); // 定义传感器1接线引脚
  sensor_1.init(&I2C_1);
  mpu6050.begin();               // 引脚已经定义，即I2C_0的引脚
  mpu6050.calcGyroOffsets(true); // 自动校正打开
}

// 移动窗口的均值滤波，平均滤波函数
float movingAverageFilter(float newValue)
{
  // 将新值添加到缓冲区
  gyroZBuffer[bufferIndex] = newValue;
  bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;
  // 计算平均值
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; ++i)
  {
    sum += gyroZBuffer[i];
  }
  return sum / MOVING_AVERAGE_WINDOW_SIZE;
}

// 转向的PWM
void angle_pwm_calculation()
{
  GyroZ = mpu6050.getGyroZ();
  float filteredGyroZ = movingAverageFilter(GyroZ); // 对角速度值进行移动平均滤波
  turn_PWM = turn_kp * (turn_spd - filteredGyroZ);
}
// 直立PID计算PWM
void verical_pwm_caculation()
{                               // 直立PID计算PWM
  AngleX = mpu6050.getAngleY(); // 陀螺仪获得X方向转动角度
  GyroX = mpu6050.getGyroY();   // 陀螺仪获得X方向角速度
  // float Keep_Angle, bias, integrate;                   // 保持角度，角度偏差，偏差积分变量
  bias = AngleX - Keep_Angle;                             // 计算角度偏差，bias为小车角度与结构静态平衡角度的差值
  integrate += bias;                                      // 偏差的积分，integrate为全局变量，一直积累
  integrate = constrain(integrate, -1000, 1000);          // 限定误差积分的最大最小值
  vertical_PWM = kp * bias + ki * integrate + kd * GyroX; // 得到PID调节后的值
                                                          /*=---通过陀螺仪返回数据计算，前倾陀螺仪X轴为正，后仰陀螺仪X轴为负。前倾车前进，后仰车后退，保持直立。
                                                          但可能为了直立，车会随时移动。*/
}

// 电机初始化
void motor_init()
{
  // 让电机链接传感器
  motor_0.linkSensor(&sensor_0); // 电机连接传感器 编码器
  // power supply voltage [V]
  driver_0.voltage_power_supply = 12;                     // 驱动器连接的电源电压
  driver_0.voltage_limit = 6;                             // 限制电压位输入电压的一半
  driver_0.init();                                        // 驱动器初始化
  motor_0.phase_resistance = 2.9;                         // 2.9 Ohms
  motor_0.torque_controller = TorqueControlType::voltage; // 电机的扭矩通过控制电压来实现
  motor_0.linkDriver(&driver_0);                          // 电机连接驱动器
  // 选择FOC控制方式 PWM
  motor_0.foc_modulation = FOCModulationType::SpaceVectorPWM; // 采用PWM方式驱动
  // motor_0.torque_controller = TorqueControlType::foc_current;  // 此驱动板没有电流采样反馈，小功率电机可以不使用电流采样
  //  设置为扭矩模式
  motor_0.controller = MotionControlType::torque; // 角度模式
  motor_0.PID_velocity.P = 0.05;                  // 速度P值，这个值不能填太大，否则容易抖动
  motor_0.PID_velocity.I = 0.001;                 // 这个值越大，响应速度会慢下来
  motor_0.PID_velocity.D = 0;
  motor_0.LPF_velocity.Tf = 0.01f; // 滤波 速度低通滤波器，越低过滤越少
  // 角度控制的PID
  motor_0.P_angle.P = 30;        // 位置PID的P值
  motor_0.velocity_limit = 20;   // 限制最大速度，弧度/秒
  motor_0.useMonitoring(Serial); // 设置使用串口进行设定
  motor_0.init();                // 电机初始化
  motor_0.initFOC();             // 传感器校正和启动FOC

  // 让电机链接传感器
  motor_1.linkSensor(&sensor_1); // 电机连接传感器 编码器
  // power supply voltage [V]
  driver_1.voltage_power_supply = 12;                     // 驱动器连接的电源电压
  driver_1.voltage_limit = 6;                             // 限制电压位输入电压的一半
  driver_1.init();                                        // 驱动器初始化
  motor_1.phase_resistance = 2.9;                         // 2.9 Ohms，电机相电阻
  motor_1.torque_controller = TorqueControlType::voltage; // 电机的扭矩通过控制电压来实现
  // link the motor and the driver
  motor_1.linkDriver(&driver_1); // 电机连接驱动器
  // 选择FOC控制方式 PWM
  motor_1.foc_modulation = FOCModulationType::SpaceVectorPWM; // 采用PWM方式驱动
  // motor_1.torque_controller = TorqueControlType::foc_current;
  //  设置为角度模式
  motor_1.controller = MotionControlType::torque; // 角度模式
  motor_1.PID_velocity.P = 0.05;                  // 速度P值，这个值不能填太大，否则容易抖动
  motor_1.PID_velocity.I = 0.001;                 // 这个值越大，响应速度会慢下来
  motor_1.PID_velocity.D = 0;
  motor_1.LPF_velocity.Tf = 0.01f; // 滤波 速度低通滤波器，越低过滤越少
  // 角度控制的PID
  motor_1.P_angle.P = 20;        // 位置PID的P值
  motor_1.velocity_limit = 20;   // 限制最大速度，弧度/秒
  motor_1.useMonitoring(Serial); // 设置使用串口进行设定
  motor_1.init();                // 电机初始化
  motor_1.initFOC();             // 传感器校正和启动FOC
}

// ====== WiFi事件函数 ======
void handleStatus()
{
  String json = "{";
  json += "\"Keep_Angle\":" + String(Keep_Angle, 3) + ",";
  json += "\"Balance_Angle_raw\":" + String(Balance_Angle_raw, 3) + ",";
  json += "\"kp\":" + String(kp, 3) + ",";
  json += "\"ki\":" + String(ki, 3) + ",";
  json += "\"kd\":" + String(kd, 3) + ",";
  json += "\"turn_kp\":" + String(turn_kp, 3) + ",";
  json += "\"flag\":\"" + String(flag) + "\",";
  json += "\"turn_spd\":" + String(turn_spd);
  json += "}";
  wifiServer.send(200, "application/json", json);
}

void handleCommand()
{
  // 获取 URL 参数 ?c= 后面的值
  String cmdStr = wifiServer.arg("c");

  if (cmdStr.length() == 0)
  {
    wifiServer.send(400, "text/plain", "Error: Missing 'c' parameter");
    return;
  }

  char DATA = cmdStr.charAt(0); // 只取第一个字符
  Serial.print("HTTP CMD: ");
  Serial.println(DATA);

  // ========== 原 switch 逻辑移植 ==========
  switch (DATA)
  {
  /*---机械平衡角度调整-----*/
  case 'u':
    Keep_Angle += 0.01;
    break;
  case 'd':
    Keep_Angle -= 0.01;
    break;

  /*----直立平衡PID调整-----*/
  case '0':
    kp -= 0.1;
    break;
  case '1':
    kp += 0.1;
    break;
  case '2':
    ki -= 0.01;
    break;
  case '3':
    ki += 0.01;
    break;
  case '4':
    kd -= 0.01;
    break;
  case '5':
    kd += 0.01;
    break;
  case '6':
    turn_kp -= 0.01;
    break;
  case '7':
    turn_kp += 0.01;
    break;

  /*-----控制程序-----*/
  case 's':
    flag = 's';
    Keep_Angle = Balance_Angle_raw;
    turn_spd = 0;
    break;
  case 'f': // 前进
    flag = 'f';
    Keep_Angle = Balance_Angle_raw + ENERGY;
    turn_spd = 0;
    break;
  case 'b': // 后退
    flag = 'b';
    Keep_Angle = Balance_Angle_raw - ENERGY;
    turn_spd = 0;
    break;
  case 'z': // 不转向
    flag = 'z';
    turn_spd = 0;
    break;
  case 'l': // 左转
    flag = 'l';
    turn_spd = turn_ENERGY;
    break;
  case 'r': // 右转
    flag = 'r';
    turn_spd = -turn_ENERGY;
    break;

  default:
    wifiServer.send(400, "text/plain", "Error: Invalid command");
    return;
  }

  // 限制 PID 参数非负
  if (kp < 0)
    kp = 0;
  if (ki < 0)
    ki = 0;
  if (kd < 0)
    kd = 0;

  // 返回成功响应
  wifiServer.send(200, "text/plain", "OK");
}
// 初始化
void setup()
{
  // IO引脚初始化
  io_init();
  // 传感器初始化
  sensor_init();
  // 电机初始化
  motor_init();
  // 使用串口进行监控设定值
  Serial.begin(115200); // 打开串口
  // command.add('L', doTarget1, "target angle0");//通过串口T命令发送位置，比如T6.28,表示电机转6.28弧度,即1圈
  // command.add('R', doTarget2, "target angle1");//通过串口T命令发送位置，比如T6.28,表示电机转6.28弧度,即1圈
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP : ");
  Serial.println(IP);
  // 绑定路由：/cmd?c=字符
  // wifiServer.on("/cmd", HTTP_GET, handleCommand());
  // 为现有路由手动添加CORS头
  wifiServer.on("/cmd", HTTP_GET, []()
                {
    wifiServer.sendHeader("Access-Control-Allow-Origin", "*");
    handleCommand(); });
  wifiServer.on("/status", HTTP_GET, []()
                {
    wifiServer.sendHeader("Access-Control-Allow-Origin", "*");
    handleStatus(); });
  // wifiServer.on("/status",HTTP_GET,  handleStatus()); // 👈 新增状态接口
  wifiServer.begin();
  // 解除wifi服务器限制，添加CORS支持
  wifiServer.onNotFound([]()
                        {
      // 处理预检请求 (OPTIONS) 和添加CORS头
      if (wifiServer.method() == HTTP_OPTIONS) {
          wifiServer.sendHeader("Access-Control-Allow-Origin", "*");
          wifiServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
          wifiServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
          wifiServer.send(204);
      } else {
          wifiServer.send(404, "text/plain", "Not Found");
      } });
  Serial.println("HTTP server started. Use http://192.168.4.1/cmd?c=...");
  Serial.println("Ready!");
  _delay(1000);
}

// 主循环
void loop()
{
  wifiServer.handleClient(); // wifi监控接收数据
  mpu6050.update(); // 陀螺仪刷新
  sensor_0.update();
  sensor_1.update();
  // 非阻塞 LED 闪烁：每 500ms 翻转一次状态
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis1 >= interval1)
  {
    previousMillis1 = currentMillis;
    digitalWrite(ledPin1, !digitalRead(ledPin1)); // 翻转 LED 状态
    Serial.print("-----Xangle->");
    Serial.print(mpu6050.getAngleX());
    // Serial.print("-----XangleSpeed->");
    // Serial.print(mpu6050.getGyroX());
    Serial.print("-----Yangle->");
    Serial.print(mpu6050.getAngleY());
    // Serial.print("-----YangleSpeed->");
    // Serial.print(mpu6050.getGyroY());
  }
  if (currentMillis - previousMillis2 >= interval2)
  {
    previousMillis2 = currentMillis;
    digitalWrite(ledPin2, !digitalRead(ledPin2)); // 翻转 LED 状态
  }

  verical_pwm_caculation();  // 直立PWM计算
  PWM = vertical_PWM;        // 电机扭矩值等于经PID计算后的值
                             // angle_pwm_calculation();  // 转向PWM计算
  motor_0.loopFOC();         // 启动，使上劲
  motor_1.loopFOC();         // 启动，使上劲

  // PWM = constrain(PWM, -60, 60); // 此时PWM还是角度值,限定在-60到60之间
  // if (PWM > 0)
  // {
  //   L_PWM = PWM + leftMotorOffset; // 加上电机死区扭矩
  //   R_PWM = PWM + rightMotorOffset;
  // }
  // if (PWM < 0)
  // {
  //   L_PWM = PWM - leftMotorOffset; // 反向转动也需要加上电机死区
  //   R_PWM = PWM - rightMotorOffset;
  // }
  // // L_PWM -= turn_PWM; // 加上转动角速度值
  // // R_PWM += turn_PWM;
  // L_PWM = constrain(L_PWM, -200, 200);
  // R_PWM = constrain(R_PWM, -200, 200);
  //   if (AngleX > 45 || AngleX < -45)
  //   { // 小车倾角过大，已倒下，停止转动；
  //       motor_0.move(0);
  //       motor_1.move(0);
  //   }
  //   else
  //   {
  //     target_angle_1 = static_cast<float>(L_PWM) / 180 * 3.14; // 把陀螺仪MPU6050获得的角度转换为弧度赋值给电机
  //     target_angle_2 = static_cast<float>(R_PWM) / 180 * 3.14;
  //     if (PWM > 0)
  //     {
  //         target_angle_1 = target_angle_1 + leftMotorOffset; // 加上电机死区扭矩
  //         target_angle_2 = target_angle_2 + rightMotorOffset;
  //     }
  //     if (PWM < 0)
  //     {
  //         target_angle_1 = target_angle_1 - leftMotorOffset; // 反向转动也需要加上电机死区
  //         target_angle_2 = target_angle_2 - rightMotorOffset;
  //     }
  //     motor_0.move(target_angle_1); // 实际这是扭矩值
  //     motor_1.move(target_angle_2);
  //   }

  motor_0.move(0.1);
  motor_1.move(0.1);
  // command.run(); // 监控输入的命令
}
