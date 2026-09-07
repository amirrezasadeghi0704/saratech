#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

#define SAMPLES 500

float raw[16];

void readAllSensors() {
  for (int i = 0; i < 16; i++) {
    float sum = 0;
    for (int s = 0; s < SAMPLES; s++) {
      digitalWrite(PA8, (i / 1) % 2);
      digitalWrite(PB1, (i / 2) % 2);
      digitalWrite(PC14, (i / 4) % 2);
      digitalWrite(PC15, (i / 8) % 2);
      delayMicroseconds(50);
      sum += 4095 - analogRead(PA0);
    }
    raw[i] = sum / SAMPLES;
    printResult();
    delay(100);
  }
}


void printResult() {
  display.setCursor(0, 0);
  display.println(raw[0]);
  display.print( : 0);
  display.println(raw[1]);
  display.print( : 1);
  display.println(raw[2]);
  display.print( : 2);
  display.println(raw[3]);
  display.print( : 3);
  display.println(raw[4]);
  display.print( : 4);
  display.println(raw[5]);
  display.print( : 5);
  display.println(raw[6]);
  display.print( : 6);
  display.println(raw[7]);
  display.print( : 7);
  display.setCursor(0, 60);
  display.println(raw[8]);
  display.print( : 8);
  display.println(raw[9]);
  display.print( : 9);
  display.println(raw[10]);
  display.print( : 10);
  display.println(raw[11]);
  display.print( : 11);
  display.println(raw[12]);
  display.print( : 12);
  display.println(raw[13]);
  display.print( : 13);
  display.println(raw[14]);
  display.print( : 14);
  display.println(raw[15]);
  display.print( : 15);
}



void setup() {
  pinMode(PA8, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);

  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("GI to SEPANTA");
  display.display();

  delay(1000);

  readAllSensors();
  printResult();
}

void loop() {
  printResult();
}