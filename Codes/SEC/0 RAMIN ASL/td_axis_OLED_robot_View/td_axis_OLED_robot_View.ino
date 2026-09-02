#include <Wire.h>
#include <TDAxis12.h>
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);

float ball_angle;
float robot_angle,robot_angle_f;
void setup() {
  // Serial.begin(115200);
  display.begin(0x3c);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();
  i2c.begin();  // Join I2C bus as master
}

void loop() {
  robot_angle = gyro.read();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(robot_angle);
  display.drawCircle(64, 32, 20, WHITE);
  display.drawLine(65 + 15 * sin(radians(robot_angle)), 32 - 15 * cos(radians(robot_angle)), 65 - 15 * sin(radians(robot_angle)), 32 + 15 * cos(radians(robot_angle)), WHITE);
  display.display();
  delay(100);
}
