#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);






//variables
int v = 45000;
int tsopMin = 0;
int tsopNum = 0;
int KF, KR, KB, KL;
int KF_S, KR_S, KB_S, KL_S;
int ldr_sens = 350;
int SHL, SHR, SHB;
int counter = 0, heading;
int buff[8];
int gy;
int d;
int SHOOT;



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
  gy = heading * 200 ;
  SHB = analogRead(PA1);
  SHR = analogRead(PA2);
  SHL = analogRead(PA3);
  SHOOT = analogRead(PA4);

  d = (SHR - SHL) * 15;
  KF = analogRead(PA5) - KF_S;
  KR = analogRead(PA6) - KR_S;
  KB = analogRead(PA7) - KB_S;
  KL = analogRead(PB0) - KL_S;
}

void printSensor() {
  display.clearDisplay();



  //gy
  display.setCursor(64, 0);
  display.print("gy");
  display.print(" = ");
  display.println(heading);

  //shoot
  display.setCursor(64, 10);
  display.print("shoot=");
  display.println(SHOOT);

  //TSOP
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


  //KAF SENSOR
  display.print("KF=");
  display.println(KF);

  display.print("KR=");
  display.println(KR);

  display.print("KB=");
  display.println(KB);

  display.print("KL=");
  display.println(KL);

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
void movesecond(int m , int n) {
  sensor();
  for (int i = 0; i < n; i++) {
    move (m);
    sensor();
    printSensor();
  }
}
void darvazeban(int a) {
  if (a == 1) {
    v = 25000;
    while (KL < ldr_sens && KR < ldr_sens) {
      move(8);
      sensor();
      printSensor();
    }
    movesecond(0, 3);
    move(16) ;
    if (SHL > SHR) {
      darvazeban(2);
    }
    else
    {
      darvazeban(3);
    }
  }
  if (a == 2) {
    v = 25000;
    while (KB < ldr_sens) {
      move(4);
      sensor();
      printSensor();
      if (tsopMin < 3900) {
        toop(tsopNum, 2);
      }

      if (KF < ldr_sens && KR < ldr_sens && KB < ldr_sens && KL < ldr_sens && SHR < SHL && SHL > 1500) {
        darvazeban(4);
      }
      else if (KF < ldr_sens && KR < ldr_sens && KB < ldr_sens && KL < ldr_sens && SHR > SHL && SHR    > 1500) {
        darvazeban(5);
      }

      if (SHB < 1700) {
        darvazeban(1);
      }
      else if (SHB > 2500) {
        darvazeban(8);
      }
    }
    if (tsopNum == 1 || tsopNum == 2 || tsopNum == 3) {
      darvazeban(7);
    }
    movesecond(12, 15);
    move(16) ;
    darvazeban(3);
  }
  if (a == 3) {
    v = 25000;
    while (KB < ldr_sens) {
      move(12);
      sensor();
      printSensor();
      if (tsopMin < 3900) {
        toop(tsopNum, 3);
      }

      if (KF < ldr_sens && KR < ldr_sens && KB < ldr_sens && KL < ldr_sens && SHR < SHL && SHL > 1500) {
        darvazeban(4);
      }
      else if (KF < ldr_sens && KR < ldr_sens && KB < ldr_sens && KL < ldr_sens && SHR > SHL && SHR > 1500) {
        darvazeban(5);
      }

      if (SHB < 1700) {
        darvazeban(1);
      }
      else if (SHB > 2500) {
        darvazeban(8);
      }

    }
    if (tsopNum == 15 || tsopNum == 14 || tsopNum == 13) {
      darvazeban(6);
    }
    movesecond(4, 15);
    move(16) ;
    darvazeban(2);
  }
  if (a == 4) {
    v = 35000;
    int brorast = 0;
    while (brorast != 2) {
      move(4);
      sensor();
      printSensor();
      if (KR < ldr_sens) {
        brorast++;
      }
    }
    if (brorast == 2) {
      darvazeban(3);
    }
  }
  if (a == 5) {
    v = 35000;
    int brochap = 0;
    while (brochap != 2) {
      move(12);
      sensor();
      printSensor();
      if (KL < ldr_sens) {
        brochap++;
      }
    }
    if (brochap == 2) {
      darvazeban(2);
    }
  }
  if (a == 6) {
    sensor();
    printSensor();
    while (tsopNum == 15 || tsopNum == 14 || tsopNum == 13) {
      sensor();
      printSensor();
      move(16);
    }
    darvazeban(2);
  }
  if (a == 7) {
    sensor();
    printSensor();
    while (tsopNum == 1 || tsopNum == 2 || tsopNum == 3) {
      sensor();
      printSensor();
      move(16);
    }
    darvazeban(3);
  }
  if (a == 8) {
    v = 25000;
    while (KL < ldr_sens && KR < ldr_sens) {
      move(0);
      sensor();
      printSensor();
    }
    movesecond(8, 3);
    move(16) ;
    if (SHL > SHR) {
      darvazeban(2);
    }
    else
    {
      darvazeban(3);
    }
  }
}

void toop(int a, int b) {
  if (a == 0 || a == 1 || a == 15) {
    sensor();
    printSensor();
    while (tsopNum == 0 && SHB > 950) {
      shoot();
      sensor();
      printSensor();
      movesecond(0, 8);
    }
    darvazeban(1);
  }
  else if (a > 1 && a < 4) {
    v = 30000;
    movesecond(4, 10);
  }
  else if (a < 15 && a > 12) {
    v = 30000;
    movesecond(12, 10);
  }
  else {
    darvazeban(b);
  }
}

void shoot()
{
  sensor();
  printSensor();
  if (SHOOT < 2000) {
    digitalWrite(PC13, 1);
    delay(200);
  }
  else {
    digitalWrite(PC13, 0);
    delay(50);
  }
}

void setup() {

  move(16);

  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);
  ///
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
  delay(500);
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

  KF_S = analogRead(PA5);
  KR_S = analogRead(PA6);
  KB_S = analogRead(PA7);
  KL_S = analogRead(PB0);


  ///////shoot
  pinMode(PC13, OUTPUT);

  
      if (SHB < 1700) {
        darvazeban(1);
      }
      else if (SHB > 2500) {
        darvazeban(8);
      }
}

void loop() {

  sensor();
  printSensor();

}
