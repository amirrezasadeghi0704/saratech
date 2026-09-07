#include <Wire.h>
#include <DHT.h>
#include <WiFi.h>
#include <ThingerESP32.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define DHTPIN 12
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define user "amirr"
#define device_Id "STARLINK"
#define device_credentials "STARLINK"

ThingerESP32 thing(user, device_Id, device_credentials);

int inbase = 1;

float ff = 0;
float fb = 0;
float fl = 0;
float fr = 0;
float fh = 0;
float ft = 0;

float fx = 0;
float fy = 0;

const int trigPins[4] = {33, 25, 32, 17}; // Front Back Left Right
const int echoPins[4] = {2, 5, 35, 16};

float distance[4] = {0, 0, 0, 0};
float distancepos[4] = {0, 0, 0, 0};
float coordinates[2] = {0, 0};

float getSpeedOfSound(float temp) {
  return 331.3 + (0.606 * temp);
}

float getDistance(int sensorIndex, float speedOfSound) {

  digitalWrite(trigPins[sensorIndex], LOW);
  delayMicroseconds(2);

  digitalWrite(trigPins[sensorIndex], HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPins[sensorIndex], LOW);

  long duration = pulseIn(echoPins[sensorIndex], HIGH, 15000);

  if (duration == 0) {
    return distance[sensorIndex];
  }

  distance[sensorIndex] =
    ((duration / 1000000.0) * speedOfSound * 100.0) / 2.0;

  return distance[sensorIndex];
}

void sensor() {

  float tem = dht.readTemperature();
  float sos = getSpeedOfSound(tem);

  for (int i = 0; i < 4; i++) {
    getDistance(i, sos);
  }
}

void pos() {

  float mabdax = (ff + fb) / 2.0;
  float mabday = (fr + fl) / 2.0;

  for (int i = 0; i < 4; i++) {
    distancepos[i] = 400.0 - distance[i];
  }

  float posx =
    ((distancepos[0] + distancepos[1]) / 2.0) - mabdax;

  float posy =
    ((distancepos[3] + distancepos[2]) / 2.0) - mabday;

  coordinates[0] = posx;
  coordinates[1] = posy;
}

void base() {

  float hum = dht.readHumidity();
  float temp = dht.readTemperature();

  if (abs(fh - hum) < 15)
    inbase = 1;
  else if (abs(ft - temp) < 7)
    inbase = 1;
  else if (abs(ff - distance[0]) < 15)
    inbase = 1;
  else if (abs(fb - distance[1]) < 15)
    inbase = 1;
  else if (abs(fl - distance[2]) < 15)
    inbase = 1;
  else if (abs(fr - distance[3]) < 15)
    inbase = 1;
  else
    inbase = 0;
}

void printall() {

  display.clearDisplay();

  display.setCursor(0, 0);
  display.print("H:");
  display.print(dht.readHumidity(), 0);
  display.print("%");

  display.setCursor(64, 0);
  display.print("T:");
  display.print(dht.readTemperature(), 0);
  display.print("C");

  display.setCursor(0, 16);
  display.print("UF:");
  display.print(distance[0], 0);

  display.setCursor(64, 16);
  display.print("UB:");
  display.print(distance[1], 0);

  display.setCursor(0, 32);
  display.print("UL:");
  display.print(distance[2], 0);

  display.setCursor(64, 32);
  display.print("UR:");
  display.print(distance[3], 0);

  display.setCursor(0, 48);
  display.print("PX:");
  display.print(coordinates[0], 0);

  display.setCursor(64, 48);
  display.print("PY:");
  display.print(coordinates[1], 0);

  display.display();
}

void setup() {

  Wire.begin();

  dht.begin();

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  thing.add_wifi("Amirreza's A15", "123456789");

  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  sensor();

  ff = distance[0];
  fb = distance[1];
  fl = distance[2];
  fr = distance[3];

  fh = dht.readHumidity();
  ft = dht.readTemperature();

  pos();

  fx = coordinates[0];
  fy = coordinates[1];

  thing["every"] >> [](pson &out) {

    out["H"] = dht.readHumidity();
    out["T"] = dht.readTemperature();

    out["UF"] = distance[0];
    out["UB"] = distance[1];
    out["UL"] = distance[2];
    out["UR"] = distance[3];

    out["PX"] = coordinates[0];
    out["PY"] = coordinates[1];

    out["BASE"] = inbase;
  };
}


void loop() {

  sensor();
  pos();
  base();
  printall();

  thing.handle();
}