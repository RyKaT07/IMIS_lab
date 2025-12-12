// ====================== KONFIGURACJA PINÓW (wg schematu str. 36) ======================
const int pinSilnikL = 5;   // PWM Lewy
const int pinSilnikP = 6;   // PWM Prawy
const int pinEnkoderL = 2;  // INT0 Lewy
const int pinEnkoderP = 3;  // INT1 Prawy

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
// Uzupełnij te wartości:
const float rozstawKol_D = 0.14;      // [m]
const float srednicaKola = 0.065;     // [m]

// ====================== REGULATOR PID (LEWY) ======================
float predkoscZadana_L = 0.0; 
float predkoscAktualna_L = 0.0;
float Kp_L = 2.0; 
float Ki_L = 5.0;   // Mniejsze Ki, bo nie możemy hamować aktywnie
float Kd_L = 0.0;
float integral_L = 0.0;
float lastError_L = 0.0;
int pwmWyjscie_L = 0;

// ====================== REGULATOR PID (PRAWY) ======================
float predkoscZadana_P = 0.0;
float predkoscAktualna_P = 0.0;
float Kp_P = 2.0; 
float Ki_P = 5.0; 
float Kd_P = 0.0;
float integral_P = 0.0;
float lastError_P = 0.0;
int pwmWyjscie_P = 0;

// ====================== TIMING ======================
const unsigned long okresPID = 50; // ms
unsigned long czasOstatniPID = 0;

const unsigned long okresWysylania = 50; // ms
unsigned long czasOstatniegoWyslania = 0;


// ====================== PRZERWANIA ENKODERÓW ======================

void przerwanieL() {
  unsigned long t = millis();
  pomiary_L[numer_L] = int(t - pomiar_L);
  pomiar_L = t;
  numer_L++;
  if (numer_L > 3) numer_L = 0;
  czasBrakuImpulsu_L = t;
}

void przerwanieP() {
  unsigned long t = millis();
  pomiary_P[numer_P] = int(t - pomiar_P);
  pomiar_P = t;
  numer_P++;
  if (numer_P > 3) numer_P = 0;
  czasBrakuImpulsu_P = t;
}

// ====================== OBLICZANIE PRĘDKOŚCI ======================

float obliczPredkosc(volatile int* tablicaPomiary) {
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

// ====================== KINEMATYKA (BEZ UJEMNYCH PRĘDKOŚCI) ======================

void ustawJazdePoLuku(float v, float Rc) {
  // Zabezpieczenie przed ujemną prędkością liniową (robot nie ma wstecznego)
  if (v < 0) v = 0;

  float vL, vP; 
  
  // Jazda prosto (bardzo duży promień)
  if (abs(Rc) > 100.0) { 
    vL = v;
    vP = v;
  } 
  // Jazda po łuku
  else {
    // Standardowy wzór na prędkości kół
    float omega = v / Rc; 
    vP = omega * (Rc - rozstawKol_D / 2.0);
    vL = omega * (Rc + rozstawKol_D / 2.0);
    
    // Zabezpieczenie sprzętowe: 
    // Jeśli geometria nakazuje kołu kręcić się do tyłu (ujemna prędkość),
    // zatrzymujemy je (0). Robot wykona wtedy najciaśniejszy możliwy skręt
    // "ciągnąc" za sobą nieruchome koło wewnętrzne.
    if (vP < 0) vP = 0;
    if (vL < 0) vL = 0;
  }

  // Przeliczenie m/s na obr/min
  float obwodKola = PI * srednicaKola;
  predkoscZadana_L = (vL * 60.0) / obwodKola;
  predkoscZadana_P = (vP * 60.0) / obwodKola;
}

// ====================== REGULATOR PID (TYLKO DODATNIE) ======================

int liczPID(float zadana, float aktualna, float &calka, float &lastErr, float kp, float ki, float kd, float dt) {
  float error = zadana - aktualna;
  
  // P
  float pTerm = kp * error;
  
  // I (z Anti-Windup 0-255)
  // Całka nie może być ujemna, bo nie możemy "oddać" sterowania ujemnym napięciem
  float integralCandidate = calka + ki * error * dt;
  if (integralCandidate < 0) integralCandidate = 0; 
  if (integralCandidate > 255) integralCandidate = 255;
  calka = integralCandidate;
  
  float iTerm = calka;

  // D
  float dTerm = 0.0;
  if (dt > 0) {
    dTerm = kd * (error - lastErr) / dt;
  }
  lastErr = error;

  float w = pTerm + iTerm + dTerm;
  
  // OGRANICZENIE TYLKO DODATNIE (0 - 255)
  if (w > 255) w = 255;
  if (w < 0)   w = 0; 
  
  return int(w);
}

void aktualizujPID() {
  unsigned long teraz = millis();
  if (teraz - czasOstatniPID < okresPID) return;
  
  float dt = (float)(teraz - czasOstatniPID) / 1000.0;
  czasOstatniPID = teraz;

  // 1. Pomiar prędkości
  predkoscAktualna_L = obliczPredkosc(pomiary_L);
  predkoscAktualna_P = obliczPredkosc(pomiary_P);

  // 2. Obliczenie PID
  pwmWyjscie_L = liczPID(predkoscZadana_L, predkoscAktualna_L, integral_L, lastError_L, Kp_L, Ki_L, Kd_L, dt);
  pwmWyjscie_P = liczPID(predkoscZadana_P, predkoscAktualna_P, integral_P, lastError_P, Kp_P, Ki_P, Kd_P, dt);

  // 3. Wysterowanie silników
  analogWrite(pinSilnikL, pwmWyjscie_L);
  analogWrite(pinSilnikP, pwmWyjscie_P);
}

// ====================== KOMUNIKACJA ======================

void obslugaKomunikacji() {
  if (!Serial.available()) return;
  char cCo = Serial.read();
  
  static float setV = 0.0; // m/s
  static float setR = 1000.0; // prosto

  switch (cCo) {
    case 's': // Bezpośrednie ustawienie RPM
      {
        float val = Serial.parseFloat();
        if(val < 0) val = 0; // Zabezpieczenie
        predkoscZadana_L = val;
        predkoscZadana_P = val;
      }
      break;
    case 'v': // Zadanie prędkości liniowej [m/s]
      {
        float val = Serial.parseFloat();
        if(val < 0) val = 0; // Robot nie ma wstecznego
        setV = val;
        ustawJazdePoLuku(setV, setR);
      }
      break;
    case 'r': // Zadanie promienia [m]
      setR = Serial.parseFloat();
      ustawJazdePoLuku(setV, setR);
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
    case '?':
      Serial.print("V_zad: "); Serial.print(setV);
      Serial.print(" R_zad: "); Serial.println(setR);
      Serial.print("PID: "); Serial.print(Kp_L); Serial.print("/");
      Serial.print(Ki_L); Serial.print("/"); Serial.println(Kd_L);
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
  
  Serial.println("Start. Brak wstecznego. Komendy: v[m/s], r[m], s[rpm]");
}

void loop() {
  obslugaKomunikacji();
  obslugaBrakuImpulsu();
  aktualizujPID();

  unsigned long teraz = millis();
  if (teraz - czasOstatniegoWyslania >= okresWysylania) {
    // Format dla Serial Plottera:
    // Oś_Zero ZadanaL AktL ZadanaP AktP PWM_L PWM_P
    Serial.print("0 "); 
    Serial.print(predkoscZadana_L); Serial.print(" ");
    Serial.print(predkoscAktualna_L); Serial.print(" ");
    Serial.print(predkoscZadana_P); Serial.print(" ");
    Serial.print(predkoscAktualna_P); Serial.print(" ");
    Serial.print(pwmWyjscie_L); Serial.print(" ");
    Serial.println(pwmWyjscie_P);
    
    czasOstatniegoWyslania = teraz;
  }
}



// ... (istniejące zmienne)

// DODAJ TO: Liczniki całkowite drogi
volatile unsigned long licznikTotal_L = 0;
volatile unsigned long licznikTotal_P = 0;

// PARAMETR DO KALIBRACJI:
// Ile impulsów enkodera przypada na 1 pełny obrót KOŁA wyjściowego?
// Szacunek na podstawie Twojego 'wspolczynnika': ~58.33 impulsu na obrót koła.
// Jeśli robot przejedzie 1m, a pokaże 0.8m -> zwiększ tę liczbę.
const float IMPULSY_NA_OBROT_KOLA = 58.33; 

// ...

// ZMODYFIKOWANE PRZERWANIA
void przerwanieL() {
  unsigned long t = millis();
  pomiary_L[numer_L] = int(t - pomiar_L);
  pomiar_L = t;
  numer_L++;
  if (numer_L > 3) numer_L = 0;
  czasBrakuImpulsu_L = t;
  
  licznikTotal_L++; // <--- DODANO
}

void przerwanieP() {
  unsigned long t = millis();
  pomiary_P[numer_P] = int(t - pomiar_P);
  pomiar_P = t;
  numer_P++;
  if (numer_P > 3) numer_P = 0;
  czasBrakuImpulsu_P = t;
  
  licznikTotal_P++; // <--- DODANO
}

// Funkcja realizująca jazdę na zadaną odległość [m]
void jedzDystans(float metry, float predkosc_ms) {
  // 1. Oblicz ile to impulsów
  float obwod = PI * srednicaKola; // w metrach
  float wymaganeObroty = metry / obwod;
  unsigned long wymaganeImpulsy = (unsigned long)(wymaganeObroty * IMPULSY_NA_OBROT_KOLA);

  // 2. Zapamiętaj stan początkowy liczników
  unsigned long startL = licznikTotal_L;
  unsigned long startP = licznikTotal_P;

  // 3. Ruszamy (zakładamy jazdę prosto, czyli R = bardzo dużo)
  ustawJazdePoLuku(predkosc_ms, 1000.0);

  Serial.print("Jade dystans: "); Serial.print(metry); 
  Serial.print("m. Cel impulsow: "); Serial.println(wymaganeImpulsy);

  // 4. Pętla oczekiwania - jedź dopóki średnia z obu kół nie osiągnie celu
  while (true) {
    // Oblicz ile przejechały
    unsigned long przebyteL = licznikTotal_L - startL;
    unsigned long przebyteP = licznikTotal_P - startP;
    unsigned long srednia = (przebyteL + przebyteP) / 2;

    // Sprawdź czy dojechaliśmy
    if (srednia >= wymaganeImpulsy) {
      break; // Wyjście z pętli
    }

    // WAŻNE: W pętli musimy aktualizować PID i obsługiwać bezpieczeństwo!
    obslugaBrakuImpulsu();
    aktualizujPID();
    
    // Opcjonalnie: wizualizacja postępu co 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      Serial.print("Postep: "); Serial.print(srednia);
      Serial.print("/"); Serial.println(wymaganeImpulsy);
      lastPrint = millis();
    }
  }

  // 5. Stop po dojechaniu
  predkoscZadana_L = 0;
  predkoscZadana_P = 0;
  pwmWyjscie_L = 0;
  pwmWyjscie_P = 0;
  analogWrite(pinSilnikL, 0);
  analogWrite(pinSilnikP, 0);
  Serial.println("Dystans osiagniety. STOP.");
}

// wewnątrz switch(cCo) w funkcji obslugaKomunikacji:

    case 'x': // Komenda: x0.5 -> jedź 0.5 metra z domyślną prędkością
      {
        float dystans = Serial.parseFloat();
        // Domyślna prędkość np. 0.2 m/s, można zmienić
        jedzDystans(dystans, 0.2); 
      }
      break;
