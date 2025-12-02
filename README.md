**Instrukcje laboratoryjne według „Limis - instrukcja Arduino 1.4.pdf”**

## Ćwiczenie 2 – Motoreduktor i pomiar prędkości
Opisane w sekcji 2 PDF („Laboratorium 2 – zadania do wykonania”). Każdy punkt odpowiada kolejnemu etapowi ćwiczenia.

1. **Podłączyć silnik przez MOSFET i uruchomić PWM (podpunkt 2.6.1 PDF)**
	- Upewnij się, że silnik podłączony jest do tranzystora MTP75N03 na pinie `pinSterowanieSilnikiem = 5`, a enkoder trafia na `pinEnkoder = 2` (INT0) lub zmien odpowiednio przypisane piny.
	- w `loop()` pozostaw `obslugaPWMzTerminala()` (możesz w tym trybie regulować PWM przez Serial).

2. **Zliczanie impulsów co sekundę (podpunkt 2.6.2 PDF)**
    - W `setup()` `2_motoreduktor.ino` odkomentuj `attachInterrupt(digitalPinToInterrupt(pinEnkoder), przerwanie, CHANGE);`
	- Zostaw włączone `pomiarImpulsowWJednejSekundzie()` w `loop()` (odkomentuj, jeśli był zakomentowany).
	- Funkcja co sekundę wypisuje `Impulsów ...` i zeruje licznik `liczbaImpulsow`, co odzwierciedla wymagania tego zadania.

3. **Pomiar czasu czterech impulsów (podpunkt 2.6.3 PDF)**
	- W pliku powyżej znajduje się alternatywna wersja `przerwanie()` (zakomentowana). Odkomentuj ją i usuń/zakomentuj poprzednią funkcje `przerwanie()`, aby mierzono odstępy czasowe pomiędzy impulsami i wypełniano tablicę `pomiary`.
    - Zakomentuj funkcje `pomiarImpulsowWJednejSekundzie()` w `loop()`, a odkomentuj `pomiarPredkosciISerialPlotter()`.
	- Sprawdź, czy wartość `wspolczynnik` zgadza się z rzeczywistością.

4. **Sekwencyjne zmiany PWM + wykres prędkości (podpunkt 2.6.4 PDF)**
	- Funkcja `sekwencjaPWM_iPredkosc()` zawiera przygotowane poziomy PWM i pomiar prędkości. W `loop()` odkomentuj jej wywołanie, oraz zakomentuj `pomiarPredkosciISerialPlotter()`, aby automatycznie zmieniać PWM i wysyłać prędkość+PWM na Serial Plotter.
	- W Serial Plotterze (baud 9600) obserwuj przebieg prędkości i błędy w zadanym czasie; dzięki temu wypełnisz wymagane 2 punkty za wizualizację zachowania silnika.

## Ćwiczenie 3 – Regulator PID z filtrem przeciw­nasyceniowym
Opisane w sekcji 3 PDF („Laboratorium 3 – zadania do wykonania”). Każdy podpunkt oznacza kolejne wymagania ćwiczenia.

1. **Pętla sprzężenia zwrotnego + regulator proporcjonalny (podpunkt 3.6.1 PDF)**
	- `3_pid.ino` od razu zestawia PWM z enkoderem, a w `aktualizujPID()` masz już człon `P` (`pTerm = Kp * error`). Pozostaw włączone przerwanie i analogWrite w `setup()`
    - zmieniamy `predkoscZadana` ręcznie przez zmiane wartosci w kodzie na górze pliku.

2. **Komunikacja i wykres na podstawie terminala (podpunkt 3.6.2 PDF)**
    - Odkomentuj w `loop()` wywołanie `obslugaKomunikacji()`, aby móc zmieniać parametry przez Serial.
	- Serial Monitor (9600) przyjmuje komendy `sX`, `pX`, `iX`, `dX`, `?`. Odsyłają one bieżące wartości zadanej prędkości i współczynników.
	- Multiplot w Serial Plotterze pokazuje wartości w formacie `0 <predkoscZadana> <predkoscAktualna> <error> <pwmWyjscie>`, co pozwala na wizualizację zachowania regulatora.

3. **Regulator PID z anti-windup (podpunkt 3.6.3 PDF)**
	- W bloku `aktualizujPID()` odkomentuj linie 158 `float iTerm = integral;` i zakomentuj/usun 157 `float iTerm = 0.0;` oraz odkomentuj linie 163 `dTerm = Kd * (error - lastError) / dt;`
	- Wartości `Kp`, `Ki`, `Kd` możesz ustawiać przez Serial, więc „dobieranie na szybko” jest możliwe bez przebudowy kodu.

4. **Przebieg dla złożonego sterowania (podpunkt 3.6.4 PDF)**
	- Odkomentuj w `loop()` wywołanie `sekwencjaPredkosci();`, aby automatycznie zmieniać `predkoscZadana` co 5 sekund pomiędzy 30, 60 i 120. Możesz też zmieniać te wartości w tablicy `predkoscPoziomy[]` na górze pliku.

5. **Dobranie optymalnych parametrów i powtórzenie eksperymentu (podpunkt 3.6.5 PDF)**
	- Dobierz wartości `Kp`, `Ki`, `Kd`, aby uzyskać jak najlepsze zachowanie regulatora i ustaw je przez Serial Monitor.

### Serial Plotter i Monitor
- Serial Plotter: `Tools > Serial Plotter`, `9600 baud`, format `0 <predkosc> <pwm>`; w zakresie ćwiczenia 3 odczytuj obie wartości równocześnie.
- Serial Monitor: `9600 baud`, włącz `No line ending` lub `Both NL & CR` zgodnie z PDF (zazwyczaj `NL`).

Jeśli coś nie działało i udało Ci się to naprawić, zrób pull request z poprawkami.
