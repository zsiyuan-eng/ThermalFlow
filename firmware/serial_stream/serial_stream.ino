/**
 * MLX90640 Thermal Sensor Data Transmitter (64Hz Version)
 * Outputs 32x24 temperature matrix to serial port for Python visualization
 * Connections: SDA=21, SCL=22
 * 64Hz refresh rate
 */

#include <Wire.h>
#include <Adafruit_MLX90640.h>

Adafruit_MLX90640 mlx;
float frame[768]; // 32 * 24 pixel array

// Performance tracking
unsigned long lastFrameTime = 0;
int frameCount = 0;
float actualFPS = 0;
bool firstFrame = true;
int errorCount = 0;

void setup() {
  // Standard baud rate, sufficient for 64Hz
  Serial.begin(460800);
  while (!Serial) delay(1);
  
  // Standard I2C speed for stability
  Wire.begin(21, 22); // ESP32 default I2C pins
  Wire.setClock(800000); // 800kHz, stable for 64Hz
  
  Serial.println("\n=== MLX90640 Thermal Sensor (64Hz Mode) ===");
  
  // Attempt sensor initialization with retry logic
  int retryCount = 0;
  while (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Sensor initialization failed, retrying...");
    delay(500);
    retryCount++;
    if (retryCount > 5) {
      Serial.println("ERROR: Cannot connect to MLX90640");
      Serial.println("Check connections: SDA->21, SCL->22, VIN->3.3V, GND->GND");
      while (1);
    }
  }
  
  // 64Hz configuration
  mlx.setMode(MLX90640_CHESS);        // Chess pattern mode
  mlx.setRefreshRate(MLX90640_16_HZ);  // 64Hz refresh rate
  
  Serial.println("SUCCESS: Sensor initialized");
  Serial.println("Mode: Chess pattern");
  Serial.println("Refresh rate: 64 Hz (15.6ms per frame)");
  Serial.println("I2C clock: 800 kHz");
  Serial.println("Serial baud rate: 460800");
  Serial.println("=== Starting data transmission ===\n");
  
  lastFrameTime = millis();
}

void loop() {
  // Get one frame of data
  int status = mlx.getFrame(frame);
  
  if (status != 0) {
    // Frame read failure
    errorCount++;
    if (errorCount % 20 == 0) {
      Serial.print("WARNING: Frame read errors: ");
      Serial.println(errorCount);
    }
    delay(10);
    return;
  }
  
  // Reset error count on successful read
  if (errorCount > 0) errorCount = 0;
  firstFrame = false;
  
  // Calculate actual frame rate (every 10 frames)
  frameCount++;
  if (frameCount >= 10) {
    unsigned long currentTime = millis();
    actualFPS = 10000.0 / (currentTime - lastFrameTime); // Average FPS over 10 frames
    lastFrameTime = currentTime;
    frameCount = 0;
  }
  
  // Send frame start marker with performance info
  Serial.print("--- Start of Frame (FPS:");
  Serial.print(actualFPS, 1);
  Serial.println(") ---");
  
  // Send 32x24 temperature data (32 values per line, space separated)
  for (int h = 0; h < 24; h++) {
    for (int w = 0; w < 32; w++) {
      float temp = frame[h * 32 + w];
      Serial.print(temp, 1); // 1 decimal place
      if (w < 31) Serial.print(" "); // Space between values
    }
    Serial.println(); // End of line
  }
  
  // Send frame end marker
  Serial.println("--- End of Frame ---");
  
  // No additional delay needed - getFrame() handles timing for 64Hz
}