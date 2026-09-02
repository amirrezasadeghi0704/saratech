#include <Adafruit_SH1106_STM32.h>  //tarif ketabkhane oled
Adafruit_SH1106 display(-1);        //faal sazi ketabkhane oled

#define ball_in_kicker !digitalRead(PA4)  // trif pa4 be onvan digital va hamchenin baaks kardan khanesh sensor

bool isball;          // tarif moteghayer 0 va 1 ie
bool OLED_EN = true;  // tarif moteghayer roshan bodan oled

int counter = 0;             // adad sahih
int buff[8];                 // araye 8 taii tashkil shode az 8 ta adad sahih
int shb, shl, shr, d;        // tarif sharp ha va difransiel
int kf, kr, kb, kl;          // tarif sensor haye kaf
int kf_s, kr_s, kb_s, kl_s;  // tarif sensor haye kaf avalie
int ldr_sens = 500;          // meghdar marzie khat sefid
int start_time = 0;          // zaman avalie estart oled
float v, vx;                 // tarif adad moteghayerie ashari
float shift;                 // tarif shift
float ballangle;             // zavie top
float balldistance;          // fasele top
float gy = 0, dgy = 0;       // kaji ezafe robot va kaji dasti ezafe shode
float tsop_cal[16] = {
  // yek araye 16 taii baraye eslah maghadir tsop
  1.0000,  // 0
  1.0482,  // 1
  1.0056,  // 2
  1.0373,  // 3
  1.0326,  // 4
  1.0147,  // 5
  1.0367,  // 6
  1.0055,  // 7
  1.0000,  // 8
  1.0000,  // 9
  1.0000,  //10
  1.0342,  //11
  1.0182,  //12
  1.0048,  //13
  1.0069,  //14
  1.0144   //15
};

void motor(int ml1, int ml2, int mr2, int mr1) {  // tabe e asli va paye ye hareket 4 ta charkh

  float correction = gy - dgy;  // ye adad ashari bara ye charkhesh be samt darvaze
  // correction baraye in dar 200 zarb mishe chon khodesh kheili kame va andaze gi tasir nadare
  ml1 += correction * 200;  // motor chap jolo
  ml2 += correction * 200;  // motor cap aghab
  mr2 += correction * 200;  // motor rast aghab
  mr1 += correction * 200;  // motor rast jolo

  if (ml1 > 65535) ml1 = 65535;    // ina baraye ine
  if (ml1 < -65535) ml1 = -65535;  // ke meghdar ha az
  if (ml2 > 65535) ml2 = 65535;    // 65535
  if (ml2 < -65535) ml2 = -65535;  // ke hadeaksar
  if (mr1 > 65535) mr1 = 65535;    // meghdar dastor
  if (mr1 < -65535) mr1 = -65535;  // PWMwrite hast
  if (mr2 > 65535) mr2 = 65535;    // kamtar ya bishtar
  if (mr2 < -65535) mr2 = -65535;  // nashe

  if (mr1 > 0) {                 // harekat padsaat gard mr1
    digitalWrite(PB12, 0);       // jahat
    pwmWrite(PB6, mr1);          // meghdar
  } else {                       // harekat saat gard mr1
    digitalWrite(PB12, 1);       // jahat
    pwmWrite(PB6, mr1 + 65535);  // meghdar + 65535 ke adad mosbat dar biad
  }

  if (mr2 > 0) {                 // harekat padsaat gard mr2
    digitalWrite(PB13, 0);       // jahat
    pwmWrite(PB7, mr2);          // meghdar
  } else {                       // harekat saat gard mr2
    digitalWrite(PB13, 1);       // jahat
    pwmWrite(PB7, mr2 + 65535);  // megdar
  }

  if (ml1 > 0) {                 // harekat padsaat gard ml1
    digitalWrite(PB15, 0);       // jahat
    pwmWrite(PB9, ml1);          // meghdar
  } else {                       // harekat saat gard ml1
    digitalWrite(PB15, 1);       // jahat
    pwmWrite(PB9, ml1 + 65535);  // meghdar
  }

  if (ml2 > 0) {                 // harekat padsaat gard ml2
    digitalWrite(PB14, 0);       // jahat
    pwmWrite(PB8, ml2);          // meghdar
  } else {                       // harekat saat gard ml2
    digitalWrite(PB14, 1);       // jahat
    pwmWrite(PB8, ml2 + 65535);  // meghdar
  }
}

void movexy(float vx, float vy, float turn = 0) {  // harekat ba bordar
  // turn yek meghdar ezafe tar tar az dgy hast baraye shot kat dar
  float ml1_move = vy + vx + turn;   // mohasebe meghdar ml1
  float ml2_move = vy - vx + turn;   // mohasebe meghdar ml2
  float mr1_move = -vy + vx - turn;  // mohasebe meghdar mr1
  float mr2_move = -vy - vx - turn;  // mohasebe meghdar mr2

  motor(ml1_move, ml2_move, mr2_move, mr1_move); // dastor harekat har 4 motor
  // // batavajoh be mahdodiat batri ma 2 ta charkh asli ro dar harekat lahaz mikonim
  // float powers[4] = {
  //   // ghodrat kham har charkh
  //   abs(ml1_move),  // ghadr motlagh
  //   abs(ml2_move),  // baraye bedast
  //   abs(mr1_move),  // omadan niroye
  //   abs(mr2_move)   // khalese har charkh
  // };

  // float vals[4] = {
  //   // magadir
  //   ml1_move,  // asli
  //   ml2_move,  // har
  //   mr1_move,  // charkh
  //   mr2_move   // ba + va -
  // };

  // int idx[4] = { 0, 1, 2, 3 };  // araye 4 taii az jens int baraye moratab ghodrat khales har charkh

  // for (int i = 0; i < 3; i++) {        // tekrar 4 bare halghe va dar enteha + 1
  //   for (int j = i + 1; j < 4; j++) {  // mes balaii vali adad dakhelesh hamihe 1 iie bishtare

  //     if (powers[idx[i]] < powers[idx[j]]) {  // moghayese ghodran khales 2 motor posht ham
  //       int tmp = idx[i];                     // age avali bishtar bod hamon lahaz mishe
  //       idx[i] = idx[j];                      // adad motor aval jaye 2 vomi ghara migire
  //       idx[j] = tmp;                         // dige oni ke ghablesh zaif tar bod ba badi moghayese nashe
  //     }                                       // ke to kar khata bendaze va dar nahayat
  //   }
  // }  // ehtemalan navazeh bod pas ino bepors ya inke adad ha ro ja gozari kon

  // float d_move[4] = { 0, 0, 0, 0 };  // harakat ghabl jagozari

  // d_move[idx[0]] = vals[idx[0]];  // motori ba bishtarin meghdar
  // d_move[idx[1]] = vals[idx[1]];  // motori ba 2 vomin mwghdar

  // motor(d_move[0], d_move[1], d_move[2], d_move[3]);  // ersal dastor harekat
}

void move(float angle) {               // tabe yaftan bordar mahz harekat
  float vx = sin(radians(angle)) * v;  // bordar harekat dar mehvar x
  float vy = cos(radians(angle)) * v;  // bordar harekat dar mehvar y
  movexy(vx, vy);                      // dastor harekat
}

void moveSpeed(float angle, float speed) {  // mes move vali ba sorat moshakhas
  float vx = sin(radians(angle)) * speed;   // bordar x zabdar sorat
  float vy = cos(radians(angle)) * speed;   // bordar y zabdar sorat
  movexy(vx, vy);                           // mes bala
}

void sensor() {  // tabe khandan tamam sensor ha
  // meghdari ke alan tavasot sensor ha khande mishavad ra az
  // meghdar avalie ke roye rang sabz daryaft mishode ra kam mikonad
  kf = analogRead(PA5) - kf_s;  // jolo
  kr = analogRead(PA6) - kr_s;  // rast
  kb = analogRead(PA7) - kb_s;  // aghab
  kl = analogRead(PB0) - kl_s;  // chap

  if ((kf > ldr_sens || kr > ldr_sens || kb > ldr_sens || kl > ldr_sens) && isball) return;  // age 4 sensor zamin
  // ro sabz tashkhis bedan az if kharej sho va edame kod ro baresi kon
  shb = analogRead(PA1);  // khandan meghdar sensor sharp aghab
  shr = analogRead(PA2);  // khandan meghdar sensor sharp rast
  shl = analogRead(PA3);  // khandan meghdar sensor sharp chap

  d = (shl - shr) * 30;  // mohasebe tafazol(difransiel) 2 sensor chap o rast

  float sumx = 0, sumy = 0;  // meghdar bordar x va y

  for (int i = 0; i < 16; i++) {  // ye tabe baaye tekrar 16 bare
    // ba tavajoh be adad haye binary { do do iie } ba in tabe 16 ta port mux ro mikhonim
    digitalWrite(PA8, (i / 1) % 2);   // hasel maghadir
    digitalWrite(PB1, (i / 2) % 2);   // bedast amade
    digitalWrite(PC14, (i / 4) % 2);  // dar in tabe adadi
    digitalWrite(PC15, (i / 8) % 2);  // bein 0 ta 15 midahad ba an sensor shomare i khande mishavad
    delayMicroseconds(2);             // 0.000002 sanie takhir baraye paidari

    float sensorangle = radians(i * 22.5);                 // moteghayeri ashari baraye bedast amadan zavie top
    float value = (4095 - analogRead(PA0)) * tsop_cal[i];  // zarb kardan tabe tasih dar meghdar khanesh sensor

    sumx += value * cos(sensorangle);  // moteghayeri baraye bedast omadan bordar x
    sumy += value * sin(sensorangle);  // moteghayeri baraye bedast omadan bordar y
  }

  ballangle = degrees(atan2(sumy, sumx));            // zavie top ba tanjant x va y
  balldistance = sqrt(pow(sumx, 2) + pow(sumy, 2));  // fasele top ba fisaghores

  if (balldistance > 2000) isball = true;  // top ke dide shod "isball" faal mishe
  else isball = false;                     // dar gheir in sorat gheir faal

  while (Serial1.available()) {  // ta har zaman etelaati az gy25 daryaft shod

    buff[counter] = Serial1.read();  // buff haye gy 25 ra bekhan

    if (counter == 0 && buff[0] != 0xAA) break;  // agar ham buff 0 bod ham serial 0xaa na bod karej sho

    counter++;  // ezafe kardan be shomaresh

    if (counter == 8) {  // age counter 8 shod

      counter = 0;  // ono 0 kon

      if (buff[0] == 0xAA && buff[7] == 0x55) {           // age ham buff[0] 0xAA bod ham buff[7] 0 x55
        gy = (int16_t)(buff[1] << 8 | buff[2]) / 100.00;  // adad bedast amade / 100 = gy
      }
    }
  }

  if (shr > 1900) dgy = 27;                    // robot rast zamin bood
  else if (shl > 1900) dgy = -30;              // robot chap zamin bood
  else if (!isball || abs(d) < 9000) dgy = 0;  // age top dide nashod ya ghadr motlagh (d) az 9000 kamtar bood

  if (millis() - start_time > 10000) OLED_EN = false;  // age 10 sanie shod ke robot roshane oledo khamoshkon
}

float clamp(float val, float min_shift, float max_shift) {  // tabe mahdod sazi
  if (val > max_shift) return max_shift;                    // az hadeaksar bishtar bood hamon hadeaksaro
  if (val < min_shift) return min_shift;                    // az hadeaghal kamtar bood hamon hadeaghal
  return val;                                               // vorodi mahdod shodero pas bede
}

void shoot() {  // tabe dastor shot

  for (int i = 0; i < 4; i++) {  // tekrar 4 bare

    digitalWrite(PC13, 1);  // roshan sodan shot

    sensor();    // khondan sensor
    printall();  // chap kardan
    out();       // farakhani tabe out
  }

  digitalWrite(PC13, 0);  // khamosh shodan shot
}

void cutshoot() {  // Beta

  if (abs(ballangle) < 18) {  // age top to rastaye darvaze bood
    shoot();                  // shot bezan
    return;                   // khoroj az if
  }

  int dir = (ballangle > 0) ? -1 : 1;  // ye jor if else baray mosbat ya manfi bodane jahat

  float turn = map(abs(ballangle), 18, 180, 10000, 24000);  // bemeghdar zavie top micharkhe
  turn = clamp(turn, 10000, 24000);                         // ziadi nacharkhe ke robot mos-bos beshe

  float vx = sin(radians(ballangle)) * 15000;  // on sinose kofti baraye + va - vojod dare
  float vy = 26000;                            // ye adad naziad na kam baraye eslah ghable kut

  for (int i = 0; i < 5; i++) {  // tekrar 5 bare ye in

    sensor();    // khondan sensor
    printall();  // chapesh

    movexy(vx, vy, dir * turn);  // ba zarb dir dar

    digitalWrite(PC13, 1);  // shot roshan

    out();  // tbe out ke vazehe

    if (kf > ldr_sens) {   // ye bar dige out jolo chon jolo robot gi male
      movesecond(180, 6);  // harekat be aghab be tedad 6 bar
      break;               // khoroj az halghe
    }
  }

  digitalWrite(PC13, 0);  // shot khamosh
  stop();                 // vazehe dige
}

void printall() {  // tabe chaap hame chi

  if (!OLED_EN) return;  // age dastor khamoshi oled ersal shod az tabe kharej sho

  display.clearDisplay();   // pak kardan safhe
  display.setCursor(0, 0);  // tanzim makan shoro be print

  display.print("BA:");        // chap kardan
  display.println(ballangle);  // chap kardan

  display.print("BD:");           // chap kardan
  display.println(balldistance);  // chap kardan

  display.print("GY:");  // chap kardan
  display.println(gy);   // chap kardan

  display.print("SHR:");  // chap kardan
  display.println(shr);   // chap kardan

  display.print("SHL:");  // chap kardan
  display.println(shl);   // chap kardan

  display.print("SHB:");  // chap kardan
  display.println(shb);   // chap kardan

  display.print("d :");  // chap kardan
  display.println(d);    // chap kardan

  display.drawCircle(93, 32, 20, WHITE);  // dastor keshidan dayere

  if (isball)                                     // dar sorat vojod top
    display.fillCircle(                           // ye dayere ye
      93 + sin(radians(ballangle + shift)) * 25,  // sefid
      32 - cos(radians(ballangle + shift)) * 25,  // ke topore
      2,                                          // va rangesh
      WHITE);                                     // sefide bekesh

  display.display();  // tadavom dar chap
}

void movesecond(int m, int n) {  // tabe haekat kotah

  sensor();  // khondan sensor

  if (!OLED_EN) n *= 30;  // age oled khamosh bod ziad she

  for (int i = 0; i < n; i++) {  // tekrar n bare
    moveSpeed(m, 50000);         // ba sorat 50000
    sensor();                    // sensor haro bekhon
    printall();                  // chap kardan
  }
}

void stop() {         // tabe tavaghof
  motor(0, 0, 0, 0);  // dastor harekat nakardan har 4 motor
}

void out() {  // tabe jolo giri az rad shodan robot az khat out

  int cnt = 0;           // tedad dafat tekrar shode
  int out_timeout = 18;  // tedad dafat mojar
  int out_sec = 8;       // tedad dafat tekrar

  if (!OLED_EN) out_timeout *= 30;  // age oled khamosh bod dafat ro ziad kon

  /////RIGHT
  if (kr > ldr_sens) {                                      // dide shodan khat sefid tavadot rast
    movesecond(-90, out_sec);                               // harekat be in tedad
    while (ballangle > 0 && isball && cnt < out_timeout) {  // tekrar 10 bare dar sorat bodan top dar rast
      sensor();                                             // khondan sensor
      printall();                                           // chaap sensor
      stop();                                               // tavaghof
      cnt++;                                                // ezafe kardan be shomaresh
    }
  }
  /////LEFT
  else if (kl > ldr_sens) {                                 // dide shodan khat sefid tavasot chap
    movesecond(90, out_sec);                                // harekat be in tedad
    while (ballangle < 0 && isball && cnt < out_timeout) {  // tekrar 10 bare dar sorat bodan top dar chap
      sensor();                                             // khondan sensor
      printall();                                           // chaap sensor
      stop();                                               // tavaghof
      cnt++;                                                // ezafe kardan be shomaresh
    }
  }
  /////BACK
  else if (kb > ldr_sens) {                                                         // dide shodan khat sefid tavasot aghab
    movesecond(0, out_sec);                                                         // harekat be in tedad
    while ((ballangle >= 90 || ballangle <= -90) && isball && cnt < out_timeout) {  // age top aghab bood 10 bar
      sensor();
      printall();  // chaap sensor
      stop();      // tavaghof
      cnt++;       // ezafe kardan be shomaresh
    }
  } else if (kf > ldr_sens) {  // dide shodan khat sefid tavasot jolo (chon moshkel dare tolani tare)

    if (ballangle > 0) {                 // age top rast bod
      for (int i = 0; i < 10; i++) {     // 10 bar tekrar
        sensor();                        // khondan sensor
        printall();                      // chaap kardan
        movexy(-18000, -10000, -12000);  // harkat aghab ghotri be jonob sharghi
      }
    }

    else {                             // age top chap
      for (int i = 0; i < 10; i++) {   // 10 bar tekrar
        sensor();                      // khondan sensor
        printall();                    // chaap kardan
        movexy(18000, -10000, 12000);  // harekat aghab ghotri be jonob gharbi
      }
    }

    while ((ballangle <= 90 && ballangle >= -90) && isball && cnt < out_timeout) {  // age top jolo bood 10 bar
      sensor();                                                                     // khondan sensor
      printall();                                                                   // chap sensor
      stop();                                                                       // tavaghof
      cnt++;                                                                        // ezafe kardan be shomaresh
    }
  }
}

void setup() {
  // tarif paye be onvan khoroji baraye tanzim jahat kharkhesh motor ha
  pinMode(PB12, OUTPUT);  // mr1
  pinMode(PB13, OUTPUT);  // mr2
  pinMode(PB14, OUTPUT);  // ml1
  pinMode(PB15, OUTPUT);  // ml2
  // tarif paye be onvan PWM baraye tanzim sorat charkhesh motor ha
  pinMode(PB6, PWM);  // mr1
  pinMode(PB7, PWM);  // mr2
  pinMode(PB8, PWM);  // ml1
  pinMode(PB9, PWM);  // ml2
  // tarif paye haye mux be onvan khoroji
  pinMode(PA8, OUTPUT);   // S0 mux
  pinMode(PB1, OUTPUT);   // S1 mux
  pinMode(PC14, OUTPUT);  // S2 mux
  pinMode(PC15, OUTPUT);  // S3 mux
  // paye analogread maghadir inja pin mode nemishe chon be sorat defalt input hast
  pinMode(PC13, OUTPUT);  // tanzim paye dastor shot be onvan khoroji

  delay(100);  // delay bara sobat

  display.begin(0x2, 0x3c);  // dastor shoro be kar oled ro I2C
  display.clearDisplay();    // dastor paksazi oled

  display.setTextSize(1);        // dastor tanzim size font
  display.setTextColor(WHITE);   // dastor tanzim rang be sefid
  display.setCursor(0, 0);       // tanzim ro pixel 0 va 0 amodi va ofoghi
  display.print("loading.26k");  // loding startup
  display.display();             // dastor baraye tadavom dar chap

  Serial1.begin(115200);  // shoro serial1 ke dovomin port USRT(UzArt) bluepile e ro bund 115200 Hz
  Serial1.write(0xA5);    // tanzim
  Serial1.write(0x54);    // baraye
  delay(750);             // delay bara tasbit
  Serial1.write(0xA5);    // shoro
  Serial1.write(0x55);    // bekar
  delay(750);             // delay bara tasbit
  Serial1.write(0xA5);    // sensor
  Serial1.write(0x52);    // gy25

  kf_s = analogRead(PA5);  // meghdar avalie ldr jolo
  kr_s = analogRead(PA6);  // meghdar avalie ldr rast
  kb_s = analogRead(PA7);  // meghdar avalie ldr aghab
  kl_s = analogRead(PB0);  // meghdar avalie ldr chap

  start_time = millis();  // shoro be shomordan zaman
}

void loop() {  // halghe tekrar bara hamishe

  sensor();    // khandan sensor
  printall();  // chap ro oled

  if (ball_in_kicker && isball) {  // age ham sensor dahane ham tsop ha top ro tashkhis dadan
    cutshoot();                    //shot bezan
  }

  else if (isball) {  // age top bood

    out();  // farakhanie out

    shift = clamp(ballangle * 1.2, -60, 60);           // robot kajtar bere soraghe top ke biofte to dahanesh
    v = map(balldistance, 10000, 3000, 30000, 55000);  // tabdile sorat bar asas fasele ye top

    move(ballangle + shift);  // ba meghdar ezafe shode ye shift boro samt top
  }

  else {
    movexy(d, (shb - 1300) * 50);  // bazgasht be darvaze
  }
}