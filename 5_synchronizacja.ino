// ====================== LAB 5 ======================
// 1) wybór punktu ćwiczenia przez odkomentowanie #define
// 2) tryb "RESET-START" (parametry w setup) – do jazdy bez USB
// 3) prosty regulator synchronizacji RS (P lub PI) do punktów 2–5

// ====================== CO OD-KOMENTOWAĆ ======================
// KROK A: wybierz czy startujesz z setup (bez USB) czy z Serial:
//   - START_FROM_SETUP = 1  -> po resecie robot zaczyna od ustawień z setup()
//   - START_FROM_SETUP = 0  -> ustawiasz v/r z Serial jak wcześniej
//
// KROK B: wybierz JEDEN punkt laboratorium (odkomentuj tylko jedną linię):
//#define LAB5_PUNKT_1_PID_DWA_SILNIKI
//#define LAB5_PUNKT_2_RS_P_LUB_PI
//#define LAB5_PUNKT_3_RS_LUK
//#define LAB5_PUNKT_4_OKRAG_POL_SZYBK
//#define LAB5_PUNKT_5_OSEMKA_ROZNE_R
//
// KROK C (opcjonalnie): jeśli punkt 2 -> wybierz RS: P albo PI:
//#define RS_TYLKO_P
//#define RS_PI

// ====================== TRYB STARTU (RESET-START vs SERIAL) ======================
#define START_FROM_SETUP 0
const unsigned long START_DELAY_MS = 600;   // po resecie poczekaj chwilę zanim ruszy (bezpieczniej)

// Parametry startowe – to jest odpowiednik tego, co wpisywałeś po Serial:
// v = prędkość liniowa [m/s], r = promień [m] (duży promień => prawie prosto)
float setup_v_ms = 0.20;
float setup_r_m  = 1000.0;

// Parametry do punktu 4 (okrąg: pół wolniej)
float setup_p4_radius_m = 0.80;
float setup_p4_v_fast_ms = 0.20;  // 1/2 okręgu
float setup_p4_v_slow_ms = 0.10;  // 2/2 okręgu

// Parametry do punktu 5 (ósemka: 2 pętle o różnych promieniach, przeciwne skręty)
float setup_p5_v_ms = 0.18;
float setup_p5_r1_m = 0.60;  // pierwsza pętla
float setup_p5_r2_m = 1.00;  // druga pętla (inna średnica)

// ====================== KONFIGURACJA PINÓW (wg schematu str. 36) ======================
const int pinSilnikL = 5;   // PWM Lewy
const int pinSilnikP = 6;   // PWM Prawy
const int pinEnkoderL = 2;  // INT0 Lewy
const int pinEnkoderP = 3;  // INT1 Prawy
const int ogrnaicz_PWM = 255;

volatile unsigned long licznikTotal_L = 0;
volatile unsigned long licznikTotal_P = 0;

// PARAMETR DO KALIBRACJI:
// Ile impulsów enkodera przypada na 1 pełny obrót KOŁA wyjściowego?
const float IMPULSY_NA_OBROT_KOLA = 58.33;

// ====================== ZMIENNE POMIAROWE (LEWY) ======================
volatile unsigned long pomiar_L = 0;
volatile int numer_L = 0;
volatile int pomiary_L[4] = {0, 0, 0, 0};
unsigned long czasBrakuImpulsu_L = 0;

// ====================== ZMIENNE POMIAROWE (PRAWY) ======================
volatile unsigned long pomiar_P = 0;
volatile int numer_P = 0;
volatile int pomiary_P[4] = {0, 0, 0, 0};
unsigned long czasBrakuImpulsu_P = 0;

// Współczynnik przeliczający czas impulsów na obr/min
const float wspolczynnik = 4114.28571429;

// ====================== PARAMETRY ROBOTA (DO ZMIERZENIA!) ======================
const float rozstawKol_D = 0.09;      // [m]
const float srednicaKola = 0.040;     // [m]

// ====================== PRĘDKOŚCI: teraz w m/s ======================
// (RPM zostawiam tylko jako diagnostykę – faktycznie regulujemy w m/s)
float vZad_L_ms = 0.0;
float vZad_P_ms = 0.0;
float vAkt_L_ms = 0.0;
float vAkt_P_ms = 0.0;

float rpmAkt_L = 0.0;
float rpmAkt_P = 0.0;

// globalne "komendy" tak jak z Serial: v oraz r
float cmdV_ms = 0.0;
float cmdR_m  = 1000.0; // prosto

// ====================== REGULATOR PID (LEWY) ======================
float Kp_L = 0.3;
float Ki_L = 2;
float Kd_L = 0.05;
float integral_L = 0.0;
float lastError_L = 0.0;
int pwmWyjscie_L = 0;

// ====================== REGULATOR PID (PRAWY) ======================
float Kp_P = 0.3;
float Ki_P = 3;
float Kd_P = 0.01;
float integral_P = 0.0;
float lastError_P = 0.0;
int pwmWyjscie_P = 0;

// ====================== TIMING ======================
const unsigned long okresPID = 50; // ms
unsigned long czasOstatniPID = 0;

const unsigned long okresWysylania = 50; // ms
unsigned long czasOstatniegoWyslania = 0;

// ====================== RS (Synchronizacja) – minimalny regulator P/PI ======================
struct RSReg {
  float kp;
  float ki;
  float integral;
  float maxCorr; // maks korekta [m/s] rozdzielana na koła
  bool useI;
};

RSReg rs = {0.6, 0.8, 0.0, 0.25, true};

// ====================== PROTOTYPY (żeby kompilator się nie czepiał kolejności) ======================
void aktualizujPID();
void obslugaBrakuImpulsu();
float sredniDystans_m();

// ====================== PRZERWANIA ENKODERÓW ======================
void przerwanieL() {
  unsigned long t = millis();
  pomiary_L[numer_L] = int(t - pomiar_L);
  pomiar_L = t;
  numer_L++;
  if (numer_L > 3) numer_L = 0;
  czasBrakuImpulsu_L = t;
  licznikTotal_L++;
}

void przerwanieP() {
  unsigned long t = millis();
  pomiary_P[numer_P] = int(t - pomiar_P);
  pomiar_P = t;
  numer_P++;
  if (numer_P > 3) numer_P = 0;
  czasBrakuImpulsu_P = t;
  licznikTotal_P++;
}

// ====================== (ZOSTAWIONE) JAZDA NA DYSTANS – jeśli używałeś tego wcześniej ======================
// Uwaga: to jest pomocnicze (nie wymagane do LAB5), ale nie usuwam, żebyś miał ciągłość.
// Działa na podstawie impulsów enkoderów (średnia z kół).
void jedzDystans(float metry, float predkosc_ms) {
  // ustaw jazdę prosto z zadaną prędkością
  cmdV_ms = predkosc_ms;
  cmdR_m  = 1000.0;

  float wymaganeMetry = metry;
  float startS = sredniDystans_m();

  Serial.print("Jade dystans: "); Serial.print(metry);
  Serial.print("m. StartS="); Serial.println(startS);

  while (true) {
    float s = sredniDystans_m() - startS;
    if (s >= wymaganeMetry) break;

    obslugaBrakuImpulsu();
    aktualizujPID();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.print("Postep[m]: "); Serial.print(s);
      Serial.print("/"); Serial.println(wymaganeMetry);
      lastPrint = millis();
    }
  }

  // STOP
  cmdV_ms = 0.0;
  vZad_L_ms = 0.0;
  vZad_P_ms = 0.0;
  analogWrite(pinSilnikL, 0);
  analogWrite(pinSilnikP, 0);
  Serial.println("Dystans osiagniety. STOP.");
}


// ====================== POMOCNICZE KONWERSJE ======================
float wheelCircumference_m() {
  return PI * srednicaKola;
}

float rpmToMs(float rpm) {
  return (rpm * wheelCircumference_m()) / 60.0;
}

float msToRpm(float ms) {
  return (ms * 60.0) / wheelCircumference_m();
}

float metersPerPulse() {
  return wheelCircumference_m() / IMPULSY_NA_OBROT_KOLA;
}

float sredniDystans_m() {
  unsigned long sum = (licznikTotal_L + licznikTotal_P) / 2;
  return (float)sum * metersPerPulse();
}

// ====================== OBLICZANIE RPM (tak jak było) ======================
float obliczRPM(volatile int* tablicaPomiary) {
  float suma = 0;
  for (int i = 0; i < 4; i++) suma += tablicaPomiary[i];
  if ((suma == 0) || (suma > 2000)) return 0.0;
  return wspolczynnik / suma;
}

void obslugaBrakuImpulsu() {
  unsigned long teraz = millis();
  if (teraz - czasBrakuImpulsu_L > 300) {
    pomiary_L[numer_L] = 999;
    numer_L++; if (numer_L > 3) numer_L = 0;
    czasBrakuImpulsu_L = teraz;
  }
  if (teraz - czasBrakuImpulsu_P > 300) {
    pomiary_P[numer_P] = 999;
    numer_P++; if (numer_P > 3) numer_P = 0;
    czasBrakuImpulsu_P = teraz;
  }
}

// ====================== KINEMATYKA (BEZ WSTECZNEGO) ======================
// Zwraca bazowe prędkości kół w m/s (przed RS)
void bazowePredkosciKol(float v, float Rc, float &vL, float &vP) {
  if (v < 0) v = 0;

  // prosto
  if (abs(Rc) > 100.0) {
    vL = v;
    vP = v;
    return;
  }

  // łuk
  float omega = v / Rc;
  vP = omega * (Rc - rozstawKol_D / 2.0);
  vL = omega * (Rc + rozstawKol_D / 2.0);

  // brak wstecznego
  if (vP < 0) vP = 0;
  if (vL < 0) vL = 0;
}

// ====================== RS: korekta aby różnica prędkości kół była jak zadana ======================
// cel: (vL - vP) ma być równy desiredDelta
void applyRS(float desiredDelta, float dt) {
  float actualDelta = vAkt_L_ms - vAkt_P_ms;
  float e = desiredDelta - actualDelta; // dodatnie => chcemy większe (L-P)

  float u = rs.kp * e;
  if (rs.useI && dt > 0) {
    rs.integral += rs.ki * e * dt;
    // proste ograniczenie
    if (rs.integral > rs.maxCorr) rs.integral = rs.maxCorr;
    if (rs.integral < -rs.maxCorr) rs.integral = -rs.maxCorr;
    u += rs.integral;
  }

  // ograniczenie korekty
  if (u > rs.maxCorr) u = rs.maxCorr;
  if (u < -rs.maxCorr) u = -rs.maxCorr;

  // rozdział korekty na oba koła (czytelne i symetryczne)
  vZad_L_ms += 0.5 * u;
  vZad_P_ms -= 0.5 * u;

  // brak wstecznego
  if (vZad_L_ms < 0) vZad_L_ms = 0;
  if (vZad_P_ms < 0) vZad_P_ms = 0;
}

// ====================== LAB 5: logika  ======================

// --- Punkt 4: okrąg, połowa wolniej ---
#if defined(LAB5_PUNKT_4_OKRAG_POL_SZYBK)
enum P4State { P4_HALF1, P4_HALF2, P4_DONE };
P4State p4State = P4_HALF1;
float p4StartDist = 0.0;
#endif

// --- Punkt 5: ósemka, różne promienie ---
#if defined(LAB5_PUNKT_5_OSEMKA_ROZNE_R)
enum P5State { P5_LOOP1, P5_LOOP2 };
P5State p5State = P5_LOOP1;
float p5StartDist = 0.0;
#endif

void lab5_updateAutonomous() {
#if defined(LAB5_PUNKT_4_OKRAG_POL_SZYBK)
  // Robimy JEDEN pełny okrąg i stop (do ponowienia – reset)
  float R = abs(setup_p4_radius_m);
  float s = sredniDystans_m() - p4StartDist;
  float halfLen = PI * R;
  float fullLen = 2.0 * PI * R;

  if (p4State == P4_HALF1) {
    cmdV_ms = setup_p4_v_fast_ms;
    cmdR_m  = setup_p4_radius_m;
    if (s >= halfLen) p4State = P4_HALF2;
  } else if (p4State == P4_HALF2) {
    cmdV_ms = setup_p4_v_slow_ms;
    cmdR_m  = setup_p4_radius_m;
    if (s >= fullLen) p4State = P4_DONE;
  } else { // DONE
    cmdV_ms = 0.0;
  }
#endif

#if defined(LAB5_PUNKT_5_OSEMKA_ROZNE_R)
  // Dwie pętle na zmianę (infinite) – reset zeruje sekwencję
  float s = sredniDystans_m() - p5StartDist;

  if (p5State == P5_LOOP1) {
    float len = 2.0 * PI * abs(setup_p5_r1_m);
    cmdV_ms = setup_p5_v_ms;
    cmdR_m  = +abs(setup_p5_r1_m);  // skręt np. w prawo
    if (s >= len) {
      p5State = P5_LOOP2;
      p5StartDist = sredniDystans_m();
    }
  } else { // LOOP2
    float len = 2.0 * PI * abs(setup_p5_r2_m);
    cmdV_ms = setup_p5_v_ms;
    cmdR_m  = -abs(setup_p5_r2_m);  // skręt w lewo (przeciwnie)
    if (s >= len) {
      p5State = P5_LOOP1;
      p5StartDist = sredniDystans_m();
    }
  }
#endif
}

void lab5_computeSetpoints(float dt) {
  float baseL = 0, baseP = 0;
  bazowePredkosciKol(cmdV_ms, cmdR_m, baseL, baseP);

  vZad_L_ms = baseL;
  vZad_P_ms = baseP;

#if defined(LAB5_PUNKT_2_RS_P_LUB_PI) || defined(LAB5_PUNKT_3_RS_LUK) || defined(LAB5_PUNKT_4_OKRAG_POL_SZYBK) || defined(LAB5_PUNKT_5_OSEMKA_ROZNE_R)
  // RS ma pilnować różnicy prędkości wynikającej z geometrii łuku
  float desiredDelta = baseL - baseP;
  applyRS(desiredDelta, dt);
#endif
}

// ====================== REGULATOR PID (funkcja jak było, tylko w m/s) ======================
int liczPID(float zadana_ms, float aktualna_ms, float &calka, float &lastErr,
            float kp, float ki, float kd, float dt) {
  float error = zadana_ms - aktualna_ms;

  float pTerm = kp * error;

  float integralCandidate = calka + ki * error * dt;
  if (integralCandidate < 0) integralCandidate = 0;
  if (integralCandidate > ogrnaicz_PWM) integralCandidate = ogrnaicz_PWM;
  calka = integralCandidate;
  float iTerm = calka;

  float dTerm = 0.0;
  if (dt > 0) dTerm = kd * (error - lastErr) / dt;
  lastErr = error;

  float w = pTerm + iTerm + dTerm;
  if (w > ogrnaicz_PWM) w = ogrnaicz_PWM;
  if (w < 0) w = 0;
  return int(w);
}

void aktualizujPID() {
  unsigned long teraz = millis();
  if (teraz - czasOstatniPID < okresPID) return;

  float dt = (float)(teraz - czasOstatniPID) / 1000.0;
  czasOstatniPID = teraz;

  // 1) pomiar RPM i konwersja na m/s
  rpmAkt_L = obliczRPM(pomiary_L);
  rpmAkt_P = obliczRPM(pomiary_P);
  vAkt_L_ms = rpmToMs(rpmAkt_L);
  vAkt_P_ms = rpmToMs(rpmAkt_P);

  // 2) w punktach autonomicznych (4/5) aktualizuj cmdV/cmdR na podstawie dystansu
  lab5_updateAutonomous();

  // 3) wylicz setpointy (w zależności od punktu)
  lab5_computeSetpoints(dt);

  // 4) PID dla obu kół
  pwmWyjscie_L = liczPID(vZad_L_ms, vAkt_L_ms, integral_L, lastError_L, Kp_L, Ki_L, Kd_L, dt);
  pwmWyjscie_P = liczPID(vZad_P_ms, vAkt_P_ms, integral_P, lastError_P, Kp_P, Ki_P, Kd_P, dt);

  analogWrite(pinSilnikL, pwmWyjscie_L);
  analogWrite(pinSilnikP, pwmWyjscie_P);
}

// ====================== KOMUNIKACJA (jak było, tylko czytelniejsze) ======================
void obslugaKomunikacji() {
  if (!Serial.available()) return;
  char cCo = Serial.read();

  switch (cCo) {
    case 'v': // prędkość liniowa [m/s]
      cmdV_ms = Serial.parseFloat();
      if (cmdV_ms < 0) cmdV_ms = 0;
      break;

    case 'r': // promień [m]
      cmdR_m = Serial.parseFloat();
      break;

    case 's': // kompatybilność: bezpośrednio zadanie kół w RPM (diagnostyka)
      {
        float rpm = Serial.parseFloat();
        if (rpm < 0) rpm = 0;
        vZad_L_ms = rpmToMs(rpm);
        vZad_P_ms = rpmToMs(rpm);
      }
      break;

    case 'p':
      { float val = Serial.parseFloat(); Kp_L = val; Kp_P = val; }
      break;
    case 'i':
      { float val = Serial.parseFloat(); Ki_L = val; Ki_P = val; }
      break;
    case 'd':
      { float val = Serial.parseFloat(); Kd_L = val; Kd_P = val; }
      break;

    case 'R': // toggle RS PI <-> P (dla punktu 2) – żebyś mógł testować jak w zadaniu
      rs.useI = !rs.useI;
      Serial.print("RS useI = "); Serial.println(rs.useI ? "PI" : "P");
      break;

    case '?':
      Serial.print("cmdV[m/s]="); Serial.print(cmdV_ms);
      Serial.print(" cmdR[m]="); Serial.print(cmdR_m);
      Serial.print(" | vZadL="); Serial.print(vZad_L_ms);
      Serial.print(" vAktL="); Serial.print(vAkt_L_ms);
      Serial.print(" | vZadP="); Serial.print(vZad_P_ms);
      Serial.print(" vAktP="); Serial.println(vAkt_P_ms);
      break;
  }
}

// ====================== SETUP i LOOP ======================
void setup() {
  Serial.begin(9600);

  pinMode(pinSilnikL, OUTPUT);
  pinMode(pinSilnikP, OUTPUT);
  pinMode(pinEnkoderL, INPUT_PULLUP);
  pinMode(pinEnkoderP, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEnkoderL), przerwanieL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEnkoderP), przerwanieP, CHANGE);

  unsigned long start = millis();
  pomiar_L = start; pomiar_P = start;
  czasBrakuImpulsu_L = start; czasBrakuImpulsu_P = start;
  czasOstatniPID = start;

  // RS: ustawienia bazowe (czytelne, łatwo zmienić)
  rs.kp = 0.6;
  rs.ki = 0.8;
  rs.maxCorr = 0.25;

#if defined(RS_TYLKO_P)
  rs.useI = false;
#elif defined(RS_PI)
  rs.useI = true;
#else
  // domyślnie: dla punktu 2 wygodnie mieć PI, ale możesz przełączyć klawiszem 'R'
  rs.useI = true;
#endif

#if defined(LAB5_PUNKT_4_OKRAG_POL_SZYBK)
  p4StartDist = sredniDystans_m();
  p4State = P4_HALF1;
#endif

#if defined(LAB5_PUNKT_5_OSEMKA_ROZNE_R)
  p5StartDist = sredniDystans_m();
  p5State = P5_LOOP1;
#endif

  Serial.println("Start. Komendy: v[m/s], r[m], p/i/d, ?  | RS toggle: 'R'");

#if START_FROM_SETUP
  // RESET-START: ustawiamy startowe cmdV/cmdR jakbyś wpisał w Serial
  delay(START_DELAY_MS);

  // Dla punktów 1–3 używamy setup_v_ms / setup_r_m
  cmdV_ms = setup_v_ms;
  cmdR_m  = setup_r_m;

  // Dla punktów 4–5 logika i tak nadpisuje cmdV/cmdR w aktualizujPID(),
  // ale ustawmy coś sensownego na start:
#if defined(LAB5_PUNKT_4_OKRAG_POL_SZYBK)
  cmdV_ms = setup_p4_v_fast_ms;
  cmdR_m  = setup_p4_radius_m;
#endif
#if defined(LAB5_PUNKT_5_OSEMKA_ROZNE_R)
  cmdV_ms = setup_p5_v_ms;
  cmdR_m  = setup_p5_r1_m;
#endif

#endif
}

void loop() {
  // Jeśli jesteś na USB -> dalej możesz zmieniać v/r na żywo
  obslugaKomunikacji();

  obslugaBrakuImpulsu();
  aktualizujPID();

  // log do Serial Plottera – zostawiam, ale w m/s (czytelniej do LAB5)
  unsigned long teraz = millis();
  if (teraz - czasOstatniegoWyslania >= okresWysylania) {
    float vAktRobot_ms = 0.5f * (vAkt_L_ms + vAkt_P_ms);
    Serial.print("vZadRobot:");     Serial.print(cmdV_ms);      Serial.print(",");
    Serial.print("vAktRobot:");Serial.print(vAktRobot_ms); Serial.print(",");
    Serial.print("vZadL:"); Serial.print(vZad_L_ms); Serial.print(",");
    Serial.print("vAktL:"); Serial.print(vAkt_L_ms); Serial.print(",");
    Serial.print("vZadP:"); Serial.print(vZad_P_ms); Serial.print(",");
    Serial.print("vAktP:"); Serial.print(vAkt_P_ms); Serial.print(",");
    Serial.print("PWM_L:"); Serial.print(pwmWyjscie_L); Serial.print(",");
    Serial.print("PWM_P:"); Serial.println(pwmWyjscie_P);
    czasOstatniegoWyslania = teraz;
  }
}
