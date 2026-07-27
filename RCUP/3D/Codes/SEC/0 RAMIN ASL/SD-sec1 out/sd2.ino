//////////////ba shoot















#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);






//variables
int v = 50000;
int tsopMin = 0;
int tsopNum = 0;
int SHL, SHR, SHB;
int counter = 0, heading;
int buff[8];
int gy;
int d;
int kf, kr, kb, kl;
int kf_s, kr_s, kb_s, kl_s;
int ldr_sens = 300;
////////////////////////////////////////////////////////
void motor(int ML1, int ML2, int MR2, int MR1) {
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


////////////////////////////////////////////////////////
void sensor() {

  tsopMin = 4095;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8, (i / 1) % 2);
    digitalWrite(PB1, (i / 2) % 2);
    digitalWrite(PC14, (i / 4) % 2);
    digitalWrite(PC15, (i / 8) % 2);

    if (analogRead(PA0) < tsopMin) {
      tsopMin = analogRead(PA0);
      tsopNum = i;
    }
  }

  //gy
  Serial1.write(0xA5);
  Serial1.write(0x51);
  while (true) {
    buff[counter] = Serial1.read();
    if (counter == 0 && buff[0] != 0xAA) break;
    counter++;
    if (counter == 8) {
      counter = 0;
      if (buff[0] == 0xAA && buff[7] == 0x55) {
        heading = (int16_t) (buff[1] << 8 | buff[2]) / 100.00;
      }
    }
  }
  gy = heading * 255 ;
  SHB = analogRead(PA1);
  SHR = analogRead(PA2);
  SHL = analogRead(PA3);

  d = (SHR - SHL) * 15;
  kf = analogRead(PA5) - kf_s;
  kr = analogRead(PA6) - kr_s;
  kb = analogRead(PA7) - kb_s;
  kl = analogRead(PB0) - kl_s;
}

////////////////////////////////////////////////
void printSensor() {
  display.clearDisplay();

  //circle


  //gy
  display.setCursor(64, 0);
  display.print("gy");
  display.print(" = ");
  display.println(heading);


  //tsop
  display.setCursor(0, 0);
  display.print(tsopNum);
  display.print(" = ");
  display.println(tsopMin);

  display.print("R= ");
  display.println(SHR);

  display.print("B= ");
  display.println(SHB);

  display.print("L= ");
  display.println(SHL);


  //LDR


  display.print("kf=");
  display.println(kf);


  display.print("kr=");
  display.println(kr);



  display.print("kb=");
  display.println(kb);



  display.print("kl=");
  display.println(kl);

  display.display();
}

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




void move_sec(int m, int n) {
  for (int i = 0; i < n; i++) {
    move(m);
    sensor();
    printSensor();
  }
}

void  out()
{
  sensor();
  printSensor();
  

  //////////front
  if (kf > ldr_sens) {
    move_sec(8, 7);
    while (tsopNum <= 4 || tsopNum >= 12) {
      sensor();
      printSensor();
      move(16);
    }
  }

    ///////////////back
  else if (kb > ldr_sens) {
    move_sec(0, 5);
    while (tsopNum >= 4 && tsopNum <= 12) {
      sensor();
      printSensor();
      move(16);
    }
  }
  
  ///////right
  else if (kr > ldr_sens) {
    move_sec(12, 5);
    while (tsopNum <= 8) {
      sensor();
      printSensor();
      move(16);
    }
  }
    //////left
  else if (kl > ldr_sens) {
    move_sec(4, 7);
    while (tsopNum >= 8) {
      sensor();
      printSensor();
      move(16);
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////
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

  //mux
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

  //gy
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54); //on
  delay(1000);
  Serial1.write(0xA5);
  Serial1.write(0x55); //ofset
  delay(1000);
  Serial1.write(0xA5);
  Serial1.write(0x51); //send data


  kf_s = analogRead(PA5) - kf_s;
  kr_s = analogRead(PA6) - kr_s;
  kb_s = analogRead(PA7) - kb_s;
  kl_s = analogRead(PB0) - kl_s;
}

void loop() {

  sensor();
  printSensor();
  out();
  if (tsopMin < 4000) {
    if (tsopNum == 0) move(0);
    if (tsopNum >= 1 && tsopNum <= 8) move(tsopNum + 2);
    else if (tsopNum >= 9 && tsopNum <= 15) move(tsopNum - 2);
  } else {
    if (SHB < 1200) motor(-v / 2 + gy - d, -v / 2 + gy + d, v / 2 + gy + d, v / 2 + gy - d);
    else if (SHB > 2000) motor(v / 2 + gy - d, v / 2 + gy + d, -v / 2 + gy + d, -v / 2 + gy - d);
    else
      move(16);
  }
}
