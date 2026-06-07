/*
 * MLX90640 热成像传感器 - WiFi版本（AP模式）
 * 
 * ESP32创建WiFi热点，Python通过WiFi连接接收数据
 * 
 * 功能：
 * 1. ESP32创建名为"MLX90640_AP"的WiFi热点
 * 2. 读取MLX90640传感器数据
 * 3. 通过TCP Socket发送温度数据
 * 4. Python客户端连接到192.168.4.1:8888接收数据
 */

#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <WiFi.h>
#include <WiFiServer.h>

// ==================== WiFi配置 ====================
const char* AP_SSID = "MLX90640_AP";       // WiFi热点名称
const char* AP_PASSWORD = "12345678";      // WiFi密码（至少8位）
const int TCP_PORT = 8888;                 // TCP端口

// ==================== 传感器配置 ====================
Adafruit_MLX90640 mlx;
float frame[768];  // 32 * 24 像素阵列

#define ROWS 24
#define COLS 32

// ==================== 全局变量 ====================
WiFiServer server(TCP_PORT);
WiFiClient client;
unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;
float currentFPS = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("  MLX90640 WiFi Streaming System");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  // ==================== 初始化I2C ====================
  Wire.begin(21, 22);
  Wire.setClock(400000);
  
  // ==================== 初始化MLX90640 ====================
  Serial.println("Initializing MLX90640...");
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("❌ ERROR: MLX90640 not found!");
    while (1) {
      delay(1000);
    }
  }
  
  // 设置传感器参数
  mlx.setMode(MLX90640_CHESS);
  mlx.setRefreshRate(MLX90640_8_HZ);  // 8Hz刷新率
  
  Serial.println("MLX90640 initialized");
  Serial.println("   Mode: CHESS");
  Serial.println("   Refresh: 8 Hz");
  
  // ==================== 创建WiFi热点 ====================
  Serial.println("\nCreating WiFi Access Point...");
  Serial.print("   SSID: ");
  Serial.println(AP_SSID);
  Serial.print("   Password: ");
  Serial.println(AP_PASSWORD);
  
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("ERROR: Failed to create AP");
    while (1) {
      delay(1000);
    }
  }
  
  // 获取IP地址（通常是192.168.4.1）
  IPAddress IP = WiFi.softAPIP();
  Serial.println("WiFi AP created");
  Serial.print("   IP Address: ");
  Serial.println(IP);
  Serial.print("   TCP Port: ");
  Serial.println(TCP_PORT);
  
  // ==================== 启动TCP服务器 ====================
  server.begin();
  Serial.println("TCP Server started");
  
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("Ready! Waiting for client connection...");
  Serial.println("Python command:");
  Serial.print("   IP: ");
  Serial.println(IP);
  Serial.print("   Port: ");
  Serial.println(TCP_PORT);
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  
  lastFrameTime = millis();
}

void loop() {
  // ==================== 检查客户端连接 ====================
  if (!client || !client.connected()) {
    // 没有客户端或连接断开，等待新连接
    client = server.accept();
    
    if (client) {
      Serial.println("\n Client connected!");
      Serial.print("   Client IP: ");
      Serial.println(client.remoteIP());
      
      // 发送欢迎消息
      client.println("MLX90640 WiFi Streaming System");
      client.println("Ready to stream thermal data...");
    } else {
      // 没有客户端，继续等待
      delay(100);
      return;
    }
  }
  
  // ==================== 读取传感器数据 ====================
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Failed to read frame");
    delay(10);
    return;
  }
  
  // ==================== 计算FPS ====================
  frameCount++;
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastFrameTime;
  
  if (elapsedTime > 0) {
    currentFPS = 1000.0 / elapsedTime;
  }
  lastFrameTime = currentTime;
  
  // ==================== 通过WiFi发送数据 ====================
  // 发送帧开始标记
  client.print("--- Start of Frame (FPS:");
  client.print(currentFPS, 1);
  client.println(") ---");
  
  // 发送24行数据
  for (int h = 0; h < ROWS; h++) {
    for (int w = 0; w < COLS; w++) {
      float temp = frame[h * COLS + w];
      
      // 发送温度值
      client.print(temp, 1);
      
      if (w < COLS - 1) {
        client.print(" ");
      }
    }
    client.println();  // 换行
  }
  
  // 发送帧结束标记
  client.println("--- End of Frame ---");
  
  // ==================== 串口调试信息 ====================
  if (frameCount % 50 == 0) {  // 每50帧打印一次
    Serial.print("Frame ");
    Serial.print(frameCount);
    Serial.print(" | FPS: ");
    Serial.print(currentFPS, 1);
    Serial.print(" | Client: ");
    Serial.println(client.connected() ? "Connected" : "Disconnected");
  }
  
  // 短暂延迟（8Hz约为125ms间隔）
  delay(10);
}
