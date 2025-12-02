// Piny i zmienne pomiarowe
const int pinSterowanieSilnikiem = 5;   // PWM
const int pinEnkoder            = 2;   // enkoder (INT0)
volatile unsigned long pomiar = 0;
volatile int numer           = 0;
volatile int pomiary[4]      = {0, 0, 0, 0};
unsigned long czasBrakuImpulsu = 0;

// Serial Plotter
const unsigned long okresWysylania = 50;
unsigned long czasOstatniegoWyslania = 0;

// Regulator PID
float predkoscZadana = 100.0; // obr/min
float Kp = 1.0;
float Ki = 0.2;
float Kd = 0.0;
float integral = 0.0;
float lastError = 0.0;
const unsigned long okresPID = 50; // ms
unsigned long czasOstatniPID = 0;
int pwmWyjscie = 0; // 0-255
float predkoscAktualna = 0.0;
float aktualnyError = 0.0;

// Sekwencja prędkości (automatyczna zmiana predkosciZadana)
const float predkoscPoziomy[] = {30.0, 60.0, 120.0};
const unsigned long czasTrwaniaPoziomuPredkosc = 5000; // ms
unsigned long czasStartPoziomuPredkosc = 0;
int aktualnyPoziomPredkosc = 0;
bool sekwencjaPredkosciAktywna = true;

const float wspolczynnik = 4114.28571429;


// ====================== POMIAR PRĘDKOŚCI ======================

// Przerwanie od enkodera – zapisuje czas między kolejnymi impulsami
void przerwanie() {
  unsigned long t = millis();
  pomiary[numer] = int(t - pomiar);  // czas od poprzedniego impulsu
  pomiar = t;
  numer++;
  if (numer > 3) numer = 0;

  czasBrakuImpulsu = t;  // ostatni czas, kiedy widzieliśmy impuls
}

// Funkcja przeliczająca czasy z tablicy na prędkość [obr/min]
float predkoscMierzona() {
  float suma = 0;
  for (int i = 0; i < 4; i++) suma += pomiary[i];

  // Zabezpieczenie – brak danych / bardzo długi okres => silnik stoi
  if ((suma == 0) || (suma > 2000)) return 0.0;

  return wspolczynnik / suma;  // obr/min
}

// Jeżeli przez dłuższy czas nie ma impulsów – „wymuszamy” prędkość = 0
void obslugaBrakuImpulsu() {
  unsigned long teraz = millis();
  if (teraz - czasBrakuImpulsu > 300) {
    // Wstawiamy dużą wartość do tablicy, żeby prędkość spadła do 0
    pomiary[numer] = 999;
    numer++;
    if (numer > 3) numer = 0;
    czasBrakuImpulsu = teraz;
  }
}

// ====================== SEKWENCJA PRĘDKOŚCI (OKRESOWA) ======================
void sekwencjaPredkosci() {
  if (!sekwencjaPredkosciAktywna) return;

  unsigned long teraz = millis();
  if (czasStartPoziomuPredkosc == 0) czasStartPoziomuPredkosc = teraz;

  if (teraz - czasStartPoziomuPredkosc >= czasTrwaniaPoziomuPredkosc) {
    aktualnyPoziomPredkosc++;
    if (aktualnyPoziomPredkosc >= (int)(sizeof(predkoscPoziomy)/sizeof(predkoscPoziomy[0]))) {
      aktualnyPoziomPredkosc = 0;
    }
    predkoscZadana = predkoscPoziomy[aktualnyPoziomPredkosc];
    Serial.print("Sekwencja predkosci - nowa predkoscZadana: ");
    Serial.println(predkoscZadana);
    czasStartPoziomuPredkosc = teraz;
  }
}


// ====================== KOMUNIKACJA UART (3.5) ======================
//
// Format polecenia z terminala:
//   sX   -> ustaw prędkość zadaną [obr/min]
//   pX   -> ustaw Kp
//   iX   -> ustaw Ki
//   dX   -> ustaw Kd
//   ?    -> wypisz aktualne parametry
//
// np. wpisujesz w terminalu (NO LINE ENDING / NL/CR zależnie jak używasz):
//   s60<ENTER>
//   p1.5<ENTER>
//   i0.2<ENTER>
//

void obslugaKomunikacji() {
  if (!Serial.available()) return;

  char cCo = Serial.read();

  switch (cCo) {
    case 'S':
    case 's':
      predkoscZadana = Serial.parseInt();   // np. s60
      break;

    case 'P':
    case 'p':
      Kp = Serial.parseFloat();             // np. p1.2
      break;

    case 'I':
    case 'i':
      Ki = Serial.parseFloat();             // np. i0.15
      break;

    case 'D':
    case 'd':
      Kd = Serial.parseFloat();             // np. d0.01
      break;

    case '?':
      Serial.print("s ");
      Serial.println(predkoscZadana);
      Serial.print("p ");
      Serial.println(Kp, 6);
      Serial.print("i ");
      Serial.println(Ki, 6);
      Serial.print("d ");
      Serial.println(Kd, 6);
      break;

    default:
      break;
  }
}


// ====================== REGULATOR PID Z ANTI-WINDUP ======================

void aktualizujPID() {
  unsigned long teraz = millis();
  if (teraz - czasOstatniPID < okresPID) return;   // jeszcze nie czas
  czasOstatniPID = teraz;

  // Czas próbkowania w sekundach (do części I i D)
  const float dt = (float)okresPID / 1000.0;

  float predkosc = predkoscMierzona();     // obr/min

  float error = predkoscZadana - predkosc; // obr/min

  predkoscAktualna = predkosc;
  aktualnyError = error;
  float pTerm = Kp * error;

  float integralCandidate = integral + Ki * error * dt;

  // filtr przeciwnasyceniowy:
  if (integralCandidate < 0 )
    {
      integral = 0;
    }
  else if ( integralCandidate > 255) 
    {
      integral = 255;
    }
  else
    {
      integral = integralCandidate;
    }
  float iTerm = 0.0;
  // float iTerm = integral;

  // 5. CZŁON D
  float dTerm = 0.0;
  if (dt > 0) {
    // dTerm = Kd * (error - lastError) / dt;
  }
  lastError = error;

  // 6. SUMA PID
  float w = pTerm + iTerm + dTerm;

  // 7. OGRANICZENIE CAŁKOWITEGO STEROWANIA DO ZAKRESU PWM [0,255]
  if (w > 255) w = 255;
  if (w < 0)   w = 0;

  pwmWyjscie = int(w);
  analogWrite(pinSterowanieSilnikiem, pwmWyjscie);
}


// ====================== SETUP i LOOP ======================

void setup() {
  Serial.begin(9600);

  // ustawienia pinów
  pinMode(pinSterowanieSilnikiem, OUTPUT);
  analogWrite(pinSterowanieSilnikiem, pwmWyjscie);

  pinMode(pinEnkoder, INPUT_PULLUP);

  // inicjalizacja zmiennych pomiarowych
  unsigned long start = millis();
  pomiar = start;
  czasBrakuImpulsu = start;
  czasOstatniegoWyslania = start;
  czasOstatniPID = start;
  numer = 0;
  for (int i = 0; i < 4; i++) pomiary[i] = 0;

  // podłącz przerwanie enkodera
  attachInterrupt(digitalPinToInterrupt(pinEnkoder), przerwanie, CHANGE);

  Serial.println("Start PID. Wpisz sX/pX/iX/dX/? w terminalu.");
}

void loop() {
  // automatyczna sekwencja prędkości (jeśli włączona)
  

  //obslugaKomunikacji();

  obslugaBrakuImpulsu();

  aktualizujPID();
  //sekwencjaPredkosci();
  // wysyłanie na Serial Plotter w regularnych odstępach
  unsigned long teraz = millis();
  if (teraz - czasOstatniegoWyslania >= okresWysylania) {
    float predkosc = predkoscMierzona();
    Serial.print("0 ");
    Serial.print(predkoscZadana);
    Serial.print(" ");
    Serial.print(predkoscAktualna);
    Serial.print(" ");
    Serial.print(aktualnyError);
    Serial.print(" ");
    Serial.println(pwmWyjscie);

    czasOstatniegoWyslania = teraz;
  }
}
