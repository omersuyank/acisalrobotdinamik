#include <Arduino.h>

// ------------ Pinler ------------
#define PUL1 2
#define DIR1 3
#define ENA1 4

#define PUL2 5
#define DIR2 6
#define ENA2 7

#define PUL3 8
#define DIR3 9
#define ENA3 10

#define PUL4 11
#define DIR4 12
#define ENA4 13

// Mega için (46-48):
#define PUL5 46
#define DIR5 47
#define ENA5 48

// ------------ Parametreler / Varsayılanlar ------------
static const uint32_t DEFAULT_STEPS_PER_REV = 1600; // 360° için adım
static const uint32_t DEFAULT_BAUD = 115200;

struct MotorPins {
  uint8_t pul, dir, ena;
};

MotorPins MOT[5] = {
  {PUL1, DIR1, ENA1},
  {PUL2, DIR2, ENA2},
  {PUL3, DIR3, ENA3},
  {PUL4, DIR4, ENA4},
  {PUL5, DIR5, ENA5}
};

// Her motor için ayrı steps/rev (dişli/mikroadım farklı olabilir)
uint32_t stepsPerRev[5] = {
  DEFAULT_STEPS_PER_REV,
  DEFAULT_STEPS_PER_REV,
  DEFAULT_STEPS_PER_REV,
  DEFAULT_STEPS_PER_REV,
  DEFAULT_STEPS_PER_REV
};

// Çalışma ayarları
int aktifMotor = 1;        // 1..5
float setAngleDeg = 15.0;  // hedef açı (derece)
float setSpeedDPS = 90.0;  // derece/saniye
uint16_t pauseMs   = 500;  // ileri-geri arasındaki bekleme
uint8_t  repeats   = 1;    // tekrar sayısı (Q için)
uint16_t rampMs    = 0;    // ivmelenme/ivmelenme süresi (ms), 0 = kapalı

// Acil durdurma bayrağı
volatile bool abortRequested = false;
volatile int movingMotor = 0; // 0 = yok, 1..5 = şu an hareket eden motor

// ------------ Yardımcılar ------------
inline void enaOn(int m)  { digitalWrite(MOT[m-1].ena, LOW);  }  // LOW = Enable (donanımına göre ters olabilir)
inline void enaOff(int m) { digitalWrite(MOT[m-1].ena, HIGH); }

inline void pulseOnce(uint8_t pulPin, uint32_t delayUs) {
  // delayUs: tek adım için toplam periyot (yaklaşık)
  // HIGH ve LOW'u simetrik tutalım
  uint32_t half = delayUs / 2;
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(half);
  digitalWrite(pulPin, LOW);
  delayMicroseconds(delayUs - half); // kalanı
}

uint32_t degToSteps(int m, float deg) {
  // negatif açı da gelebilir; adım sayısının mutlağını döndürürüz
  float stepsPerDeg = (float)stepsPerRev[m-1] / 360.0f;
  float raw = fabs(deg) * stepsPerDeg;
  long s = lroundf(raw);
  if (s < 0) s = 0;
  return (uint32_t)s;
}

bool directionFor(float deg) {
  // pozitif açı = ileri (HIGH) kabulü
  return (deg >= 0.0f);
}

uint32_t dpsToDelayUs(float dps, int m) {
  // derece/saniye -> step/saniye -> mikro-saniye periyot
  if (dps <= 0.0f) dps = 1.0f;
  float stepsPerDeg = (float)stepsPerRev[m-1] / 360.0f;
  float sps = dps * stepsPerDeg;     // steps per second
  if (sps < 1.0f) sps = 1.0f;
  float periodUs = 1000000.0f / sps; // µs / adım
  if (periodUs < 2.0f) periodUs = 2.0f; // çok uç durum koruması
  return (uint32_t)periodUs;
}

// İvmelenmeli hareket (lineer rampa). rampMs=0 ise sabit hız.
void moveWithProfile(int m, float deg, float dps, uint16_t rampMs) {
  MotorPins p = MOT[m-1];
  bool fwd = directionFor(deg);
  uint32_t totalSteps = degToSteps(m, deg);
  if (totalSteps == 0) return;

  digitalWrite(p.dir, fwd ? HIGH : LOW);
  enaOn(m);
  movingMotor = m;
  abortRequested = false; // her yeni hareket öncesi temizle
  Serial.print(F("MOVING M")); Serial.print(m); Serial.println(F(" START"));

  uint32_t tgtDelayUs = dpsToDelayUs(dps, m);

  if (rampMs == 0 || totalSteps < 10) {
    // Basit: sabit hız
    for (uint32_t i = 0; i < totalSteps; i++) {
      if (abortRequested) break;
      pulseOnce(p.pul, tgtDelayUs);
    }
  } else {
    // Trapez profili (yaklaşık):
    // ramp adımlarını süreye göre tahmin edelim
    // sps_target = 1e6 / tgtDelayUs
    float spsTarget = 1000000.0f / (float)tgtDelayUs;
    // ramp sırasında kat edilecek adım ~ spsTarget * (rampMs/1000) / 2 (hız ortalama büyür/azalır)
    uint32_t rampSteps = (uint32_t)max(1.0f, (spsTarget * (rampMs / 1000.0f)) * 0.5f);
    if (rampSteps * 2 > totalSteps) rampSteps = totalSteps / 2;

    // Başlangıç gecikmesini hedefin ~%150'si yapalım (daha yumuşak kalkış)
    uint32_t startDelayUs = (uint32_t)(tgtDelayUs * 1.5f);
    if (startDelayUs < tgtDelayUs + 2) startDelayUs = tgtDelayUs + 2;

    // 1) Hızlanma
    for (uint32_t i = 0; i < rampSteps; i++) {
      if (abortRequested) break;
      float t = (rampSteps <= 1) ? 1.0f : (float)i / (float)(rampSteps - 1);
      // lineer interpolasyon: start -> target
      uint32_t curDelay = (uint32_t)(startDelayUs + t * ( (int)tgtDelayUs - (int)startDelayUs ));
      pulseOnce(p.pul, curDelay);
    }

    // 2) Sabit hız orta bölüm
    uint32_t midSteps = totalSteps - rampSteps - rampSteps;
    for (uint32_t i = 0; i < midSteps; i++) {
      if (abortRequested) break;
      pulseOnce(p.pul, tgtDelayUs);
    }

    // 3) Yavaşlama (ters yönde interpolasyon)
    for (uint32_t i = 0; i < rampSteps; i++) {
      if (abortRequested) break;
      float t = (rampSteps <= 1) ? 1.0f : (float)i / (float)(rampSteps - 1);
      uint32_t curDelay = (uint32_t)(tgtDelayUs + t * ( (int)startDelayUs - (int)tgtDelayUs ));
      pulseOnce(p.pul, curDelay);
    }
  }
  movingMotor = 0;
  Serial.print(F("MOVING M")); Serial.print(m); Serial.println(F(" END"));
}

// Kullanışlı sarmalayıcılar
void doForward(int m) { moveWithProfile(m,  fabs(setAngleDeg), setSpeedDPS, rampMs); }
void doBackward(int m){ moveWithProfile(m, -fabs(setAngleDeg), setSpeedDPS, rampMs); }

// ------------ Komut okuma / yorumlama ------------
String inbuf;

void printHelp() {
  Serial.println(F("Komutlar:"));
  Serial.println(F("  M <1-5>         : Motor sec"));
  Serial.println(F("  A <deg>         : Aci (derece, +/-)"));
  Serial.println(F("  V <dps>         : Hiz (derece/s)"));
  Serial.println(F("  S <steps_rev>   : Secili motor icin steps/rev ata (ornegin 1600, 3200)"));
  Serial.println(F("  RAMP <ms>       : Ivmelenme/yavaslama suresi (0 = kapali)"));
  Serial.println(F("  PAUSE <ms>      : Ileri-geri arasi bekleme"));
  Serial.println(F("  REP <n>         : Q icin tekrar sayisi"));
  Serial.println(F("  G               : Yalniz ileri hareket (setAngleDeg, setSpeedDPS)"));
  Serial.println(F("  Q               : Ileri-geri hareket (REP kadar tekrar)"));
  Serial.println(F("  SHOW            : Mevcut ayarlari yaz"));
  Serial.println(F("  MOVE m deg dps  : Tek satir ileri hareket"));
  Serial.println(F("  TEST m deg dps rep pause_ms : Ileri-geri tekrarla"));
  Serial.println(F("  ABORT           : Acil durdur (hareketi kes, ENA OFF)"));
  Serial.println(F("  ENA [m] ON|OFF  : ENA ac/kapat (m verilirse sadece o motor)"));
  Serial.println(F("  STOP [m]        : O anki hareketi kes (m verilirse yalniz o motor)"));
  Serial.println(F("  STATUS          : Tum motorlarin ENA/MOVING durumunu yaz"));
}

void showState() {
  Serial.println(F("--------- AYARLAR ---------"));
  Serial.print(F("Motor: ")); Serial.println(aktifMotor);
  Serial.print(F("Aci (deg): ")); Serial.println(setAngleDeg, 3);
  Serial.print(F("Hiz (deg/s): ")); Serial.println(setSpeedDPS, 3);
  Serial.print(F("Steps/Rev (secili): ")); Serial.println(stepsPerRev[aktifMotor-1]);
  Serial.print(F("Rampa (ms): ")); Serial.println(rampMs);
  Serial.print(F("Pause (ms): ")); Serial.println(pauseMs);
  Serial.print(F("Repeat: ")); Serial.println(repeats);
  float stepsPerDeg = (float)stepsPerRev[aktifMotor-1]/360.0f;
  Serial.print(F("1 deg = ")); Serial.print(stepsPerDeg, 3); Serial.println(F(" step"));
  Serial.println(F("---------------------------"));
}

void parseAndExec(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Büyük-küçük farketmesin
  String u = line; u.toUpperCase();

  // Tek kelimelikler
  if (u == "HELP" || u == "?") { printHelp(); return; }
  if (u == "SHOW") { showState(); return; }
  if (u == "G") { doForward(aktifMotor); return; }

  if (u == "ABORT") {
    abortRequested = true;
    // Tum suruculerin ENA'sini kapat
    for (int i=1;i<=5;i++) enaOff(i);
    movingMotor = 0;
    Serial.println(F("ABORT: hareket durduruldu, ENA OFF"));
    return;
  }

  if (u == "Q") {
    for (uint8_t i=0; i<repeats; i++) {
      doForward(aktifMotor);
      delay(pauseMs);
      doBackward(aktifMotor);
      if (i+1 < repeats) delay(pauseMs);
    }
    return;
  }

  // Tokenize
  // Not: Arduino String parsing sade tutuldu.
  char buf[64];
  line.toCharArray(buf, sizeof(buf));
  char *cmd = strtok(buf, " ,\t");
  if (!cmd) return;

  if (!strcmp(cmd, "M")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      int m = atoi(p);
      if (m >= 1 && m <= 5) {
        aktifMotor = m;
        Serial.print(F("Motor secildi: ")); Serial.println(aktifMotor);
        enaOn(aktifMotor); // secili motoru ac
      }
    }
    return;
  }

  if (!strcmp(cmd, "ENA")) {
    // ENA [m] ON/OFF
    char *pm = strtok(NULL, " ,\t");
    char *pstate = strtok(NULL, " ,\t");
    int m = 0;
    if (pm && isdigit(*pm)) {
      m = atoi(pm);
    } else {
      pstate = pm; // ilk token ON/OFF ise
    }
    if (!pstate) return;
    bool turnOn = !strcasecmp(pstate, "ON");
    bool turnOff = !strcasecmp(pstate, "OFF");
    if (!(turnOn || turnOff)) return;
    if (m >= 1 && m <= 5) {
      if (turnOn) { enaOn(m); Serial.print(F("ENA ON M")); Serial.println(m); }
      else { enaOff(m); Serial.print(F("ENA OFF M")); Serial.println(m); }
    } else {
      for (int i=1;i<=5;i++) {
        if (turnOn) enaOn(i); else enaOff(i);
      }
      Serial.println(turnOn ? F("ENA: ON (tum motorlar)") : F("ENA: OFF (tum motorlar)"));
    }
    return;
  }

  if (!strcmp(cmd, "STOP")) {
    // STOP [m]
    char *pm = strtok(NULL, " ,\t");
    if (pm) {
      int m = atoi(pm);
      if (m >= 1 && m <= 5) {
        if (movingMotor == m) {
          abortRequested = true;
          movingMotor = 0;
        }
        enaOff(m);
        Serial.print(F("STOP: M")); Serial.println(m);
      }
    } else {
      abortRequested = true;
      for (int i=1;i<=5;i++) enaOff(i);
      movingMotor = 0;
      Serial.println(F("STOP: tum motorlar"));
    }
    return;
  }

  if (!strcmp(cmd, "STATUS")) {
    for (int i=1;i<=5;i++) {
      bool enaOnState = (digitalRead(MOT[i-1].ena) == LOW); // LOW=Enable
      bool isMoving = (movingMotor == i);
      Serial.print(F("STATUS M")); Serial.print(i);
      Serial.print(F(" ENA:")); Serial.print(enaOnState ? F("ON") : F("OFF"));
      Serial.print(F(" MOVING:")); Serial.println(isMoving ? 1 : 0);
    }
    return;
  }

  if (!strcmp(cmd, "A")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      setAngleDeg = atof(p);
      Serial.print(F("Aci(deg): ")); Serial.println(setAngleDeg, 3);
    }
    return;
  }

  if (!strcmp(cmd, "V")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      setSpeedDPS = atof(p);
      if (setSpeedDPS <= 0) setSpeedDPS = 1.0f;
      Serial.print(F("Hiz(deg/s): ")); Serial.println(setSpeedDPS, 3);
    }
    return;
  }

  if (!strcmp(cmd, "S")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      uint32_t spr = (uint32_t) atol(p);
      if (spr >= 50 && spr <= 200000) {
        stepsPerRev[aktifMotor-1] = spr;
        Serial.print(F("Steps/Rev (M")); Serial.print(aktifMotor); Serial.print(F("): "));
        Serial.println(spr);
      }
    }
    return;
  }

  if (!strcmp(cmd, "RAMP")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      int v = atoi(p);
      if (v < 0) v = 0;
      if (v > 5000) v = 5000;
      rampMs = (uint16_t)v;
      Serial.print(F("Rampa (ms): ")); Serial.println(rampMs);
    }
    return;
  }

  if (!strcmp(cmd, "PAUSE")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      int v = atoi(p);
      if (v < 0) v = 0;
      pauseMs = (uint16_t)v;
      Serial.print(F("Pause (ms): ")); Serial.println(pauseMs);
    }
    return;
  }

  if (!strcmp(cmd, "REP")) {
    char *p = strtok(NULL, " ,\t");
    if (p) {
      int v = atoi(p);
      if (v < 1) v = 1;
      if (v > 200) v = 200;
      repeats = (uint8_t)v;
      Serial.print(F("Repeat: ")); Serial.println(repeats);
    }
    return;
  }

  // Kısa yollar
  if (!strcmp(cmd, "MOVE")) {
    char *pm = strtok(NULL, " ,\t");
    char *pd = strtok(NULL, " ,\t");
    char *pv = strtok(NULL, " ,\t");
    if (pm && pd && pv) {
      int m = atoi(pm);
      float deg = atof(pd);
      float dps = atof(pv);
      if (m >= 1 && m <= 5 && dps > 0.0f) {
        aktifMotor = m;
        abortRequested = false;
        moveWithProfile(m, deg, dps, rampMs);
      }
    }
    return;
  }

  if (!strcmp(cmd, "TEST")) {
    // TEST m deg dps rep pause_ms
    char *pm = strtok(NULL, " ,\t");
    char *pd = strtok(NULL, " ,\t");
    char *pv = strtok(NULL, " ,\t");
    char *pr = strtok(NULL, " ,\t");
    char *pp = strtok(NULL, " ,\t");
    if (pm && pd && pv && pr && pp) {
      int m = atoi(pm);
      float deg = atof(pd);
      float dps = atof(pv);
      int rep = atoi(pr);
      int pms = atoi(pp);
      if (m>=1 && m<=5 && dps>0 && rep>=1) {
        aktifMotor = m;
        abortRequested = false;
        for (int i=0;i<rep;i++) {
          if (abortRequested) break;
          moveWithProfile(m,  fabs(deg), dps, rampMs);
          if (abortRequested) break;
          delay(pms);
          moveWithProfile(m, -fabs(deg), dps, rampMs);
          if (i+1<rep) delay(pms);
        }
      }
    }
    return;
  }

  Serial.println(F("Bilinmeyen komut. 'HELP' yaz."));
}

// ------------ Setup / Loop ------------
void setup() {
  for (int i=0;i<5;i++) {
    pinMode(MOT[i].pul, OUTPUT);
    pinMode(MOT[i].dir, OUTPUT);
    pinMode(MOT[i].ena, OUTPUT);
    // Varsayım: ENA LOW = aktif
    digitalWrite(MOT[i].ena, LOW);
    digitalWrite(MOT[i].dir, LOW);
    digitalWrite(MOT[i].pul, LOW);
  }
  Serial.begin(DEFAULT_BAUD);
  delay(200);
  Serial.println(F("Stepper Test Konsolu hazir. 'HELP' yazin."));
  showState();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      parseAndExec(inbuf);
      inbuf = "";
    } else {
      if (inbuf.length() < 120) inbuf += c;
    }
  }
}
