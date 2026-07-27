#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

// ================== تنظیمات قابل تنظیم ==================
int motorPower = 80000;
int motorTurnSpeed = 60000;  // سرعت کمتر برای چرخش دقیق‌تر
int tsopMin = 0, tsopNum = 0;
int heading = 0, gy = 0, d = 0;
int SHL, SHR, SHB;
int buff[8], counter = 0;

// آستانه‌های بهینه‌شده
#define TSOP_THRESHOLD 3200  // آستانه بهتر برای تشخیص توپ
#define TSOP_SHOOT_MIN 7     // محدوده شوت: 7-9 (جلوی مرکزی)
#define TSOP_SHOOT_MAX 9
#define HEADING_TOLERANCE 10  // تلرانس زاویه برای تصحیح

// آستانه‌های شارپ
#define SHARP_THRESHOLD 1500  // آستانه تشخیص دیوار
#define SHARP_TOO_FAR 500     // خیلی دور از دروازه
#define SHARP_TARGET 1200     // فاصله ایده‌آل از دروازه

// ----------- سنسورهای LDR ---------------
#define LDR_FRONT   PA4
#define LDR_BACK    PA5
#define LDR_LEFT    PA6
#define LDR_RIGHT   PA7
int LDR_Threshold = 2000;

// ----------- شوتر و اسپین‌بک ------------
#define SHOOTER_PIN PB4
#define SPINBACK_PIN PB5
#define DRIBBLER_SPEED 220  // سرعت بهینه دریبلر

// ----------- تایمرها و حالت‌ها ---------------
unsigned long lastShootTime = 0;
#define SHOOT_COOLDOWN 1000  // حداقل 1 ثانیه بین شوت‌ها
bool ballLocked = false;
bool returningToGoal = false;  // حالت بازگشت به دروازه

// ================== کنترل موتور بهبود یافته ==================
void motor(int ML1, int ML2, int MR2, int MR1) {
  ML1 = constrain(ML1, -65535, 65535);
  ML2 = constrain(ML2, -65535, 65535);
  MR2 = constrain(MR2, -65535, 65535);
  MR1 = constrain(MR1, -65535, 65535);
  
  // اعمال تصحیح ژایرو برای حفظ جهت
  int correction = gy / 10;
  ML1 -= correction; ML2 -= correction;
  MR1 += correction; MR2 += correction;
  
  digitalWrite(PB12, (MR1 <= 0));
  pwmWrite(PB6, (MR1 > 0) ? MR1 : MR1 + 65535);
  digitalWrite(PB13, (MR2 <= 0));
  pwmWrite(PB7, (MR2 > 0) ? MR2 : MR2 + 65535);
  digitalWrite(PB14, (ML2 <= 0));
  pwmWrite(PB8, (ML2 > 0) ? ML2 : ML2 + 65535);
  digitalWrite(PB15, (ML1 <= 0));
  pwmWrite(PB9, (ML1 > 0) ? ML1 : ML1 + 65535);
}

// ================== خواندن سنسورها با فیلتر ==================
void readSensors() {
  // TSOP با میانگین‌گیری برای کاهش نویز
  tsopMin = 4095;
  int readings[16];
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8,  (i >> 0) & 1);
    digitalWrite(PB1,  (i >> 1) & 1);
    digitalWrite(PC14, (i >> 2) & 1);
    digitalWrite(PC15, (i >> 3) & 1);
    delayMicroseconds(10);  // زمان تثبیت مالتی‌پلکسر
    readings[i] = analogRead(PA0);
    if (readings[i] < tsopMin) { 
      tsopMin = readings[i]; 
      tsopNum = i; 
    }
  }
  
  // قطب‌نما
  Serial1.write(0xA5); Serial1.write(0x51);
  unsigned long timeout = millis() + 50;
  while (millis() < timeout) {
    if (Serial1.available()) {
      buff[counter] = Serial1.read();
      if (counter == 0 && buff[0] != 0xAA) break;
      counter++;
      if (counter == 8) {
        counter = 0;
        if (buff[0] == 0xAA && buff[7] == 0x55)
          heading = (int16_t)(buff[1] << 8 | buff[2]) / 100.0;
        break;
      }
    }
  }
  gy = heading * 190;
  
  SHB = analogRead(PA1);
  SHR = analogRead(PA2);
  SHL = analogRead(PA3);
  d = (SHR - SHL) * 15;
}

// بررسی خروج از زمین با هیسترزیس
bool isOutOfField() {
  static bool wasOut = false;
  int frontVal = analogRead(LDR_FRONT);
  int backVal = analogRead(LDR_BACK);
  int leftVal = analogRead(LDR_LEFT);
  int rightVal = analogRead(LDR_RIGHT);
  
  bool isOut = (frontVal > LDR_Threshold || backVal > LDR_Threshold ||
                leftVal > LDR_Threshold || rightVal > LDR_Threshold);
  
  if (isOut) wasOut = true;
  else if (wasOut && frontVal < LDR_Threshold - 200 && 
           backVal < LDR_Threshold - 200 &&
           leftVal < LDR_Threshold - 200 && 
           rightVal < LDR_Threshold - 200) {
    wasOut = false;
  }
  return wasOut;
}

// ================== حرکت‌های بهبود یافته ==================
void stopRobot() { 
  motor(0, 0, 0, 0); 
}

void moveTowardBall() {
  int speed = motorTurnSpeed;
  
  // چرخش هوشمند بر اساس موقعیت توپ
  if (tsopNum >= TSOP_SHOOT_MIN && tsopNum <= TSOP_SHOOT_MAX) {
    // توپ جلوی مرکز - حرکت مستقیم
    motor(motorPower, motorPower, -motorPower, -motorPower);
  } else if (tsopNum < TSOP_SHOOT_MIN) {
    // توپ سمت چپ
    motor(-speed, -speed, speed, speed);
  } else {
    // توپ سمت راست
    motor(speed, speed, -speed, -speed);
  }
}

void performShoot() {
  if (millis() - lastShootTime > SHOOT_COOLDOWN) {
    digitalWrite(SHOOTER_PIN, HIGH);
    delay(250);  // شوت سریع‌تر
    digitalWrite(SHOOTER_PIN, LOW);
    lastShootTime = millis();
    stopRobot();
    delay(200);  // استراحت بعد از شوت
  }
}

// بررسی نیاز به بازگشت به دروازه
bool shouldReturnToGoal() {
  // اگر هر دو شارپ چپ و راست دیوار نمی‌بینند = خیلی دور رفته
  return (SHL < SHARP_TOO_FAR && SHR < SHARP_TOO_FAR);
}

// بازگشت به موقعیت دروازه
void returnToGoal() {
  returningToGoal = true;
  
  // اگر شارپ عقب دیوار می‌بیند = نزدیک دروازه است
  if (SHB > SHARP_THRESHOLD) {
    // تراز کردن با دروازه بر اساس شارپ چپ و راست
    int sharpDiff = SHL - SHR;
    
    if (abs(sharpDiff) < 100) {
      // تراز است - جلو بیا
      motor(motorPower * 0.6, motorPower * 0.6, -motorPower * 0.6, -motorPower * 0.6);
    } else if (sharpDiff > 0) {
      // چپ نزدیک‌تر - به راست برو
      motor(motorPower * 0.5, 0, -motorPower * 0.5, 0);
    } else {
      // راست نزدیک‌تر - به چپ برو
      motor(0, motorPower * 0.5, 0, -motorPower * 0.5);
    }
    
    // اگر به موقعیت مناسب رسید
    if (SHL > SHARP_TARGET && SHR > SHARP_TARGET) {
      returningToGoal = false;
      stopRobot();
    }
  } else {
    // هنوز دیوار عقب را نمی‌بیند - به سمت دروازه حرکت کن
    // استفاده از قطب‌نما برای حرکت رو به عقب
    if (abs(heading - 180) < 20 || abs(heading + 180) < 20) {
      // رو به دروازه - مستقیم برو
      motor(-motorPower * 0.7, -motorPower * 0.7, motorPower * 0.7, motorPower * 0.7);
    } else if (heading > 0) {
      // چرخش به چپ برای رسیدن به 180 درجه
      motor(-motorPower * 0.5, -motorPower * 0.5, motorPower * 0.5, motorPower * 0.5);
    } else {
      // چرخش به راست برای رسیدن به 180 درجه
      motor(motorPower * 0.5, motorPower * 0.5, -motorPower * 0.5, -motorPower * 0.5);
    }
  }
}

// ================== راه‌اندازی ==================
void setup() {
  stopRobot();
  pinMode(PB12, OUTPUT); pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  pinMode(PB9, PWM); pinMode(PB8, PWM); 
  pinMode(PB7, PWM); pinMode(PB6, PWM);
  pinMode(PA8, OUTPUT); pinMode(PB1, OUTPUT); 
  pinMode(PC14, OUTPUT); pinMode(PC15, OUTPUT);
  
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0); 
  display.print("DEFENDER V2.0");
  display.setCursor(0, 16); 
  display.print("Initializing...");
  display.display();
  
  Serial1.begin(115200);
  Serial1.write(0xA5); Serial1.write(0x54); delay(1000);
  Serial1.write(0xA5); Serial1.write(0x55); delay(1000);
  Serial1.write(0xA5); Serial1.write(0x51);
  
  pinMode(SHOOTER_PIN, OUTPUT);
  pinMode(SPINBACK_PIN, PWM);
  
  display.clearDisplay();
  display.setCursor(0, 0); 
  display.print("READY!");
  display.display();
  delay(1000);
}

// ================== حلقه اصلی بهبود یافته ==================
void loop() {
  readSensors();
  
  // نمایش وضعیت روی OLED
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 200) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("TSOP:"); display.print(tsopNum);
    display.print(" V:"); display.print(tsopMin);
    display.setCursor(0, 12);
    display.print("H:"); display.print(heading);
    display.setCursor(0, 24);
    display.print("SL:"); display.print(SHL);
    display.print(" SR:"); display.print(SHR);
    display.setCursor(0, 36);
    if (returningToGoal) display.print("RETURN");
    else if (ballLocked) display.print("LOCKED");
    else display.print("SEARCH");
    display.display();
    lastDisplayUpdate = millis();
  }
  
  // بررسی خط
  if (isOutOfField()) {
    stopRobot();
    analogWrite(SPINBACK_PIN, 0);
    ballLocked = false;
    returningToGoal = false;
    delay(100);
    return;
  }
  
  // بررسی نیاز به بازگشت به دروازه (اولویت بالا)
  if (shouldReturnToGoal() && !ballLocked) {
    analogWrite(SPINBACK_PIN, 0);
    returnToGoal();
    return;
  }
  
  // اگر در حال بازگشت و توپ دید، قطع کن بازگشت
  if (returningToGoal && tsopMin < TSOP_THRESHOLD) {
    returningToGoal = false;
  }
  
  // منطق اصلی
  if (tsopMin < TSOP_THRESHOLD) {
    analogWrite(SPINBACK_PIN, DRIBBLER_SPEED);
    ballLocked = true;
    
    // شرط شوت: توپ جلو + جهت صحیح
    if (tsopNum >= TSOP_SHOOT_MIN && tsopNum <= TSOP_SHOOT_MAX) {
      if (abs(heading) < HEADING_TOLERANCE) {
        performShoot();
      } else {
        // تصحیح جهت قبل از شوت
        if (heading > 0) motor(-30000, -30000, 30000, 30000);
        else motor(30000, 30000, -30000, -30000);
      }
    } else {
      moveTowardBall();
    }
  } else {
    // توپ دیده نمی‌شود
    analogWrite(SPINBACK_PIN, 0);
    ballLocked = false;
    
    // چرخش جستجو
    if (abs(heading) > 90) {
      motor(-40000, -40000, 40000, 40000);
    } else {
      motor(40000, 40000, -40000, -40000);
    }
  }
}