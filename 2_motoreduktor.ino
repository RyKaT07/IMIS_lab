const int pinSterowanieSilnikiem = 5;   // pin PWM do tranzystora MOSFET 
const int pinEnkoder            = 2;   // wejscie z enkodera
unsigned long liczbaImpulsow = 0;
unsigned long czasStartPomiaru = 0;

int wypelnieniePWM = 150;


volatile unsigned long czas   = 0;      // aktualny czas (ms)
volatile unsigned long pomiar = 0;      // czas ostatniego impulsu (ms)
volatile int numer           = 0;       // indeks w tablicy
volatile int pomiary[4]      = {0, 0, 0, 0};  // czasy kolejnych 4 impulsow (ms)

const unsigned long okresWysylania = 100;
unsigned long czasOstatniegoWyslania = 0;

const int poziomyPWM[3] = {80, 150, 220};  // możesz zmienić wg uznania
const unsigned long czasTrwaniaPoziomu = 5000; // ms, czas jednego poziomu

int aktualnyPoziom = 0;
unsigned long czasStartPoziomu = 0;

void obslugaPWMzTerminala() {
  if (Serial.available() > 0) {
    int nowaWartosc = Serial.parseInt();

    if (nowaWartosc >= 0 && nowaWartosc <= 255) {
      wypelnieniePWM = nowaWartosc;
      analogWrite(pinSterowanieSilnikiem, wypelnieniePWM);

      Serial.print("PWM = ");
      Serial.println(wypelnieniePWM);
    } else {
      Serial.println("Blad: podaj liczbe z zakresu 0–255.");
    }
  }
}

void przerwanie() {
  liczbaImpulsow++;
}

void pomiarImpulsowWJednejSekundzie() {
  unsigned long teraz = millis();

  if (teraz - czasStartPomiaru >= 1000) {
    noInterrupts();
    unsigned long impulsowWOkresie = liczbaImpulsow;
    liczbaImpulsow = 0;
    interrupts();

    Serial.print("Impulsow w ostatniej sekundzie: ");
    Serial.println(impulsowWOkresie);

    czasStartPomiaru = teraz;
  }
}


/*
void przerwanie() {
  czas = millis();
  pomiary[numer] = int(czas - pomiar);  // czas od poprzedniej zmiany stanu
  pomiar = czas;
  numer++;
  if (numer > 3) numer = 0;
}
*/

float predkoscMierzona() {
  float suma = 0;
  for (int i = 0; i < 4; i++) suma += pomiary[i];
  const float wspolczynnik = 4114.28571429;

  return ((suma == 0) || (suma > 2000)) ? 0 : wspolczynnik / suma;
}


void pomiarPredkosciISerialPlotter() {
  czas = millis();
  if (czas - pomiar > 300) {
    pomiary[numer] = 999;
    numer++;
    if (numer > 3) numer = 0;
    pomiar = czas;
  }

  if (czas - czasOstatniegoWyslania >= okresWysylania) {
    float predkosc = predkoscMierzona();   // obr/min
    
    Serial.print("0 ");
    Serial.println(predkosc);

    czasOstatniegoWyslania = czas;
  }
}

void sekwencjaPWM_iPredkosc() {
  unsigned long teraz = millis();

  if (teraz - czasStartPoziomu >= czasTrwaniaPoziomu) {
    aktualnyPoziom++;
    if (aktualnyPoziom >= 3) {
      aktualnyPoziom = 0;
    }

    wypelnieniePWM = poziomyPWM[aktualnyPoziom];
    analogWrite(pinSterowanieSilnikiem, wypelnieniePWM);

    czasStartPoziomu = teraz;
  }

  czas = teraz;
  if (czas - pomiar > 300) {
    pomiary[numer] = 999;
    numer++;
    if (numer > 3) numer = 0;
    pomiar = czas;
  }
  
  if (czas - czasOstatniegoWyslania >= okresWysylania) {
    float predkosc = predkoscMierzona();   // obr/min


    Serial.print("0 ");
    Serial.print(predkosc);
    Serial.print(" ");
    Serial.println(wypelnieniePWM);

    czasOstatniegoWyslania = czas;
  }
}


void setup() {
  Serial.begin(9600);

  // sterowanie silnikiem
  pinMode(pinSterowanieSilnikiem, OUTPUT);
  analogWrite(pinSterowanieSilnikiem, wypelnieniePWM);

  // wejscie z enkodera
  pinMode(pinEnkoder, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(pinEnkoder), przerwanie, CHANGE);
  // czasStartPomiaru = millis();

  Serial.println("Start. Podaj wartosc PWM 0–255 (NO LINE ENDING).");
}

void loop() {
  obslugaPWMzTerminala();

  // pomiarImpulsowWJednejSekundzie();

  // pomiarPredkosciISerialPlotter();

  // sekwencjaPWM_iPredkosc();
}
