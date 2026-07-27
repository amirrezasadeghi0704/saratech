#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

// ---------- پارامترهای میدان و ایمنی (سیستم سانتی‌متر) ----------
const float FIELD_LENGTH_CM = 235.0;   // طول زمین (از دروازهٔ خود تا دروازهٔ حریف)
const float FIELD_WIDTH_CM  = 142.0;   // عرض زمین
const float DESIRED_BACK_DIST_CM = 45.0; // فاصلهٔ مطلوب از عقب (یعنی مکان ایستادن)
const float SIDE_LIMIT_CM = 10.0;      // خط اوت: 10 سانت
const float PENALTY_DEPTH_CM = 25.0;   // از دروازه تا انتهای محوطهٔ جریمه (اطلاعات شما)
const float OUT_BUFFER_CM = 30.0;      // بافر از خط اوت برای جلوگیری (طبق خواسته: 30 سانت از اوت؟)
                                       // (از متن شما: "30 سانت فاصله از خط اوت" -> کاربرد به عنوان حاشیه ایمنی)
                                       
// ---------- پارامترهای سرعت و کالیبراسیون سنسورها ----------
int v_base = 65000; // سرعت پایه (همان v قدیمی). می‌توانید کمتر بگیرید برای تست.
const float MAX_LAT_ERR_CM = FIELD_WIDTH_CM / 2.0; // بیشینه خطا در عرض
const float MIN_SPEED_SCALE = 0.18; // حداقل نسبت سرعت هنگام نزدیک شدن زیاد به مرکز

// --- پارامترهای کالیبراسیون: تبدیل خوانش آنالوگ (0-4095) به سانتی‌متر ---
// اینها را باید کالیبره کنی: اندازه‌گیری کن وقتی سنسور نزدیک/دور است و عدد مناسب بذار
const int CAL_SH_MIN = 200;    // خوانش آنالوگ حالت خیلی نزدیک (مثال)
const int CAL_SH_MAX = 3800;   // خوانش آنالوگ حالت خیلی دور (مثال)
const float CAL_DIST_MIN_CM = 5.0;   // نزدیک‌ترین فاصله که سنسور گزارش می‌دهد
const float CAL_DIST_MAX_CM = 200.0; // دورترین فاصلهٔ مفید سنسور (تنظیم کن)

// ---------- متغیرهای وضعیت و حسگر ----------
int v = v_base;
int tsopMin = 0;
int tsopNum = 0;
int SHL, SHR, SHB;
int counter = 0, heading;
int buff[8];
int gy;
int d;

enum RobotState { SEEK_BALL, GO_TO_GOAL, POSITION_CENTER, AVOID_OUT, IN_PENALTY_RETURN };
RobotState state = SEEK_BALL;

// ---------- توابع سخت‌افزاری قبلی شما (بدون تغییر ساختاری زیاد) ----------
void motor(long ML1, long ML2, long MR2, long MR1) {
  if (ML1 > 65535) ML1 = 65535;
  if (ML2 > 65535) ML2 = 65535;
  if (MR2 > 65535) MR2 = 65535;
  if (MR1 > 65535) MR1 = 65535;

  if (ML1 < -65535) ML1 = -65535;
  if (ML2 < -65535) ML2 = -65535;
  if (MR2 < -65535) MR2 = -65535;
  if (MR1 < -65535) MR1 = -65535;

  //MR1
  if (MR1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, MR1);
  } else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, MR1 + 65535);
  }

  //MR2
  if (MR2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, MR2);
  } else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, MR2 + 65535);
  }

  //ML2
  if (ML2 > 0) {
    digitalWrite(PB14, 0);
    pwmWrite(PB8, ML2);
  } else {
    digitalWrite(PB14, 1);
    pwmWrite(PB8, ML2 + 65535);
  }

  //ML1
  if (ML1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PB9, ML1);
  } else {
    digitalWrite(PB15, 1);
    pwmWrite(PB9, ML1 + 65535);
  }
}

// ------------ توابع کمک‌کننده -------------
float mapSensorToCm(int raw) {
  raw = constrain(raw, CAL_SH_MIN, CAL_SH_MAX);
  // نگاشت خطی از خوانش به سانتی‌متر (تقریبی — کالیبراسیون لازم)
  float t = float(raw - CAL_SH_MIN) / float(CAL_SH_MAX - CAL_SH_MIN);
  float dist = CAL_DIST_MIN_CM + t * (CAL_DIST_MAX_CM - CAL_DIST_MIN_CM);
  return dist;
}

// تابعی برای بدست آوردن موقعیت تقریبی عرضی (x از سمت چپ) و فاصله از عقب (y)
void computePositionApprox(float &x_cm, float &y_back_cm) {
  // فرض: SHL فاصله به دیوارهٔ چپ، SHR فاصله به دیوارهٔ راست، SHB فاصله به عقب
  float leftDist  = mapSensorToCm(SHL);
  float rightDist = mapSensorToCm(SHR);
  float backDist  = mapSensorToCm(SHB);

  // روش میانگین: x_from_left = leftDist، x_from_right = FIELD_WIDTH - rightDist
  float x1 = leftDist;
  float x2 = FIELD_WIDTH_CM - rightDist;
  x_cm = (x1 + x2) / 2.0;

  // y_back_cm = فاصله تا دیوارهٔ عقب (پشت ربات) — اگر پشت = 0 در عقب زمین
  y_back_cm = backDist;
}

// تبدیل زاویه/جهت به یکی از 16 حالت حرکت شما و فراخوانی move(sector)
// ما از همان تابع move(m) شما استفاده می‌کنیم:
void moveSectorByAngle(float angleDeg, long vScaled) {
  // angleDeg: 0 = روبه جلو (به سمت دروازه حریف)، مثبت به سمت راست
  // تبدیل به بخش 0..15
  int sector = int(round(angleDeg / 22.5)) % 16;
  if (sector < 0) sector += 16;
  // استفاده از move(sector) موجود — ولی ما باید مقدار v را جهانی تنظیم کنیم
  // برای ساده‌سازی: تغییر مقدار v (متغیر global) قبل از فراخوانی move
  int oldv = v;
  v = vScaled;
  move(sector);
  v = oldv;
}

// ------------- بازنویسی sensor() و printSensor() با خواندن جدید --------------
void sensor() {
  tsopMin = 4095;
  tsopNum = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8, (i / 1) % 2);
    digitalWrite(PB1, (i / 2) % 2);
    digitalWrite(PC14, (i / 4) % 2);
    digitalWrite(PC15, (i / 8) % 2);

    int a = analogRead(PA0);
    if (a < tsopMin) {
      tsopMin = a;
      tsopNum = i;
    }
  }

  // gy خواندن (همان روند قبلی با سریال)
  Serial1.write(0xA5);
  Serial1.write(0x51);
  // خواندن داده از سریال (غیرمسدود کننده)
  int available = Serial1.available();
  while (available > 0) {
    int b = Serial1.read();
    // جمع‌آوری فریم 8 بایتی
    buff[counter] = b;
    if (counter == 0 && buff[0] != 0xAA) {
      // اگر اولین بایت نبود، بازنشانی
      counter = 0;
      // بخاطر سادگی شکسته و ادامه میدیم
    } else {
      counter++;
      if (counter == 8) {
        counter = 0;
        if (buff[0] == 0xAA && buff[7] == 0x55) {
          int16_t rawH = (int16_t)((buff[1] << 8) | buff[2]);
          heading = int(rawH / 100.0);
        }
      }
    }
    available = Serial1.available();
  }

  gy = heading * 190;
  SHB = analogRead(PA1);
  SHR = analogRead(PA2);
  SHL = analogRead(PA3);

  d = (SHR - SHL) * 15; // مقدار شما (می‌تونی حذف یا تنظیمش کنی)
}

void printSensor(float x_cm = -1, float y_back_cm = -1) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("gy: "); display.println(heading);

  display.print("TSOP num: "); display.print(tsopNum); display.print(" min: "); display.println(tsopMin);

  display.print("SHL: "); display.print(SHL); 
  if (x_cm >= 0) display.print("  x(cm): "), display.print(x_cm);
  display.println();

  display.print("SHR: "); display.print(SHR); 
  display.println();

  display.print("SHB: "); display.print(SHB);
  if (y_back_cm >= 0) display.print("  back(cm): "), display.print(y_back_cm);
  display.println();

  // نمایش وضعیت فعلی ربات
  display.print("State: ");
  switch (state) {
    case SEEK_BALL: display.println("SEEK_BALL"); break;
    case GO_TO_GOAL: display.println("GO_TO_GOAL"); break;
    case POSITION_CENTER: display.println("POSITION_CENTER"); break;
    case AVOID_OUT: display.println("AVOID_OUT"); break;
    case IN_PENALTY_RETURN: display.println("IN_PENALTY_RETURN"); break;
  }

  display.display();
}

// ------------ الگوریتم تصمیم‌گیری / حرکت -------------
float distance2D(float x1, float y1, float x2, float y2) {
  float dx = x1 - x2;
  float dy = y1 - y2;
  return sqrt(dx*dx + dy*dy);
}

// هدف نهایی: موقعیت مرکزی عرض و 45 سانت از عقب
void goToTarget(float target_x_cm, float target_y_back_cm, bool scaleSpeedByLatErr) {
  float x_cm, y_back_cm;
  computePositionApprox(x_cm, y_back_cm);

  // مختصات هدف و فاصله
  float dist = distance2D(x_cm, y_back_cm, target_x_cm, target_y_back_cm);

  // سرعت مقیاس‌شده: اگر از مرکز خارج شدی سرعت کم کن
  float latErr = fabs(target_x_cm - x_cm);
  float speedScale = 1.0 - constrain(latErr / MAX_LAT_ERR_CM, 0.0, 0.9);
  if (!scaleSpeedByLatErr) speedScale = 1.0;
  if (speedScale < MIN_SPEED_SCALE) speedScale = MIN_SPEED_SCALE;
  long vScaled = (long)(v_base * speedScale);

  // اگر خیلی نزدیک شدی، توقف کن (مثلاً کمتر از 3 سانت)
  if (dist < 3.0) {
    move(16); // توقف موتور (تعریف شده در کد شما)
    return;
  }

  // جهت به سمت هدف: محاسبه زاویه نسبت به محور جلو (y)
  // delta_x = هدف.x - فعلی.x (به راست مثبت)
  // delta_y = هدف.y - فعلی.y (به جلو مثبت)
  float dx = target_x_cm - x_cm;
  float dy = target_y_back_cm - y_back_cm;
  // اگر dy==0 و dx==0 قبلاً handled شد
  // angleDeg: 0 روبه جلو، مثبت به راست
  float angleRad = atan2(dx, dy);
  float angleDeg = angleRad * 180.0 / 3.14159265;
  if (angleDeg < 0) angleDeg += 360.0;

  // فراخوانی بر اساس سکتور
  moveSectorByAngle(angleDeg, vScaled);
}

// انتخاب مسیر جایگزین ساده وقتی به نزدیک خط اوت رسیدیم
void avoidOutStrategy(float x_cm, float y_back_cm) {
  // حرکت مورب به داخل زمین تا به فاصلهٔ امن برسیم
  float safeX = constrain(x_cm, SIDE_LIMIT_CM + OUT_BUFFER_CM, FIELD_WIDTH_CM - SIDE_LIMIT_CM - OUT_BUFFER_CM);
  float safeY = max(y_back_cm, DESIRED_BACK_DIST_CM); // حرکت به جلو کمی
  goToTarget(safeX, safeY, true);
}

// بررسی ورود به محوطهٔ پنالتی حریف (فاصله از دروازهٔ حریف)
bool inOpponentPenalty(float x_cm, float y_back_cm) {
  // فرض می‌کنیم دروازهٔ حریف در انتهای طول زمین (y = FIELD_LENGTH)
  // ولی y_back_cm اندازه از عقب زمین است؛ پس موقعیت y در سیستم ما = FIELD_LENGTH - y_back_cm
  float y_from_own_goal = FIELD_LENGTH_CM - y_back_cm;
  // اگر y_from_own_goal نزدیک به انتهای زمین (بیشتر از FIELD_LENGTH - PENALTY_DEPTH) باشد
  return (y_from_own_goal > FIELD_LENGTH_CM - PENALTY_DEPTH_CM - 1.0);
}

// وقتی وارد محوطهٔ جریمه شدی و توپ همراهته: بازگشت به سمت دروازهٔ خود
void returnFromPenalty() {
  // هدف: بازگشت به وسط عرض و فاصلهٔ 45 سانت از عقب (مکان ایمنی)
  float target_x = FIELD_WIDTH_CM / 2.0;
  float target_y = DESIRED_BACK_DIST_CM;
  goToTarget(target_x, target_y, true);
}

// ------------- حلقهٔ اصلی با state machine -------------
void setup() {
  move(16);

  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);

  pinMode(PB9, PWM);
  pinMode(PB8, PWM);
  pinMode(PB7, PWM);
  pinMode(PB6, PWM);

  pinMode(PA8, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);

  //oled
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("CALCULATE");
  display.display();
  delay(400);
  display.setTextSize(1);

  //gyro
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54); //on
  delay(1000);
  Serial1.write(0xA5);
  Serial1.write(0x55); //offset
  delay(1000);
  Serial1.write(0xA5);
  Serial1.write(0x51); //send data
}

void loop() {
  sensor();

  // محاسبه موقعیت تقریبی
  float x_cm, y_back_cm;
  computePositionApprox(x_cm, y_back_cm);

  // وضعیت ورود یا عدم ورود توپ
  bool ballSeen = (tsopMin < 3700);

  // بررسی مرزها (اوُت)
  bool nearLeftOut  = (x_cm < SIDE_LIMIT_CM + 10.0);
  bool nearRightOut = (x_cm > FIELD_WIDTH_CM - SIDE_LIMIT_CM - 10.0);
  bool nearOut = nearLeftOut || nearRightOut;

  // بررسی ورود به محوطهٔ پنالتی حریف
  bool inPenalty = inOpponentPenalty(x_cm, y_back_cm);

  // تصمیم‌گیری حالت
  if (inPenalty && ballSeen) {
    state = IN_PENALTY_RETURN;
  } else if (nearOut && !ballSeen) {
    state = AVOID_OUT;
  } else if (ballSeen) {
    state = SEEK_BALL;
  } else {
    // اگر توپ نداری، برو به هدف (دروازهٔ حریف) و سپس ایست در موقعیت مرکزی
    // ما دو مرحله: اول نزدیک شدن به دروازه (GO_TO_GOAL)، سپس ایست در مرکز (POSITION_CENTER)
    // ساده‌سازی: اگر خیلی نزدیک به نقطهٔ هدف نباشی => GO_TO_GOAL، در غیر این صورت POSITION_CENTER
    float goal_x = FIELD_WIDTH_CM / 2.0;
    float goal_y_back = FIELD_LENGTH_CM - 10.0; // نزدیک دروازهٔ حریف (مثال 10cm از انتها)
    float distToGoal = distance2D(x_cm, y_back_cm, goal_x, goal_y_back);
    if (distToGoal > 20.0) state = GO_TO_GOAL;
    else state = POSITION_CENTER;
  }

  // اجرای حالت‌ها
  switch (state) {
    case SEEK_BALL: {
      // وقتی توپ دیده شد: از tsopNum استفاده کن که جهت توپ رو نشون میده (0..15)
      // در کد اصلی اگر tsopMin<3700 و tsopNum==0 ... شما قبلاً با map حرکت می‌کردی.
      // برای سادگی: استفاده مستقیم از move با ترجمهٔ قبلی شما:
      if (tsopNum == 0) move(0);
      else if (tsopNum >= 1 && tsopNum <= 8) move(tsopNum + 2);
      else if (tsopNum >= 9 && tsopNum <= 15) move(tsopNum - 2);
      // همچنین نمایش
      printSensor(x_cm, y_back_cm);
      break;
    }

    case GO_TO_GOAL: {
      // هدف: نزدیک شدن به دروازهٔ حریف
      float goal_x = FIELD_WIDTH_CM / 2.0;
      float goal_y_back = FIELD_LENGTH_CM - 10.0; // 10cm مانده به انتها
      goToTarget(goal_x, goal_y_back, true);
      printSensor(x_cm, y_back_cm);
      break;
    }

    case POSITION_CENTER: {
      // هدف: وسط عرض و فاصله 45 از عقب
      float target_x = FIELD_WIDTH_CM / 2.0;
      float target_y = DESIRED_BACK_DIST_CM;
      goToTarget(target_x, target_y, true);
      printSensor(x_cm, y_back_cm);
      break;
    }

    case AVOID_OUT: {
      avoidOutStrategy(x_cm, y_back_cm);
      printSensor(x_cm, y_back_cm);
      break;
    }

    case IN_PENALTY_RETURN: {
      // با توپ در محوطهٔ جریمه: بازگشت به نقطهٔ ایمن وسط
      returnFromPenalty();
      printSensor(x_cm, y_back_cm);
      break;
    }
  }

  delay(30); // حلقه سریع ولی نه خیلی سریع
}

// ---------- تابع move (همان تابع شما) -----------
void move(int m) {
  if (m == 0) motor(v + gy, v + gy, -v + gy, -v + gy);
  else if (m == 1) motor(v + gy, v / 2 + gy, -v + gy, -v / 2 + gy);
  else if (m == 2) motor(v + gy, 0 + gy, -v + gy, 0 + gy);
  else if (m == 3) motor(v + gy, -v / 2 + gy, -v + gy, v / 2 + gy);
  else if (m == 4) motor(v + gy, -v + gy, -v + gy, v + gy);
  else if (m == 5) motor(v / 2 + gy, -v + gy, -v / 2 + gy, v + gy);
  else if (m == 6) motor(0 + gy, -v + gy, 0 + gy, v + gy);
  else if (m == 7) motor(-v / 2 + gy, -v + gy, v / 2 + gy, v + gy);
  else if (m == 8) motor(-v + gy, -v + gy, v + gy, v + gy);
  else if (m == 9) motor(-v + gy, -v / 2 + gy, v + gy, v / 2 + gy);
  else if (m == 10) motor(-v + gy, 0 + gy, v + gy, 0 + gy);
  else if (m == 11) motor(-v + gy, v / 2 + gy, v + gy, -v / 2 + gy);
  else if (m == 12) motor(-v + gy, v + gy, v + gy, -v + gy);
  else if (m == 13) motor(-v / 2 + gy, v + gy, v / 2 + gy, -v + gy);
  else if (m == 14) motor(0 + gy, v + gy, 0 + gy, -v + gy);
  else if (m == 15) motor(v / 2 + gy, v + gy, -v / 2 + gy, -v + gy);
  else if (m == 16) motor(0 + gy, 0 + gy, 0 + gy, 0 + gy);
}
