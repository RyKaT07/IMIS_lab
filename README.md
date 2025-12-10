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

---

## Ćwiczenie 4 – Mini-robot o napędzie różnicowym
Opisane w sekcji 4 PDF („Laboratorium 4 – zadania do wykonania”). [cite_start]Ćwiczenie łączy dwa silniki w jeden system sterowania z kinematyką ruchu po łuku[cite: 788].



1. **Konfiguracja i pomiar parametrów robota (podpunkt 4.1 i 4.2 PDF)**
   - W pliku `4_robot.ino` znajdź sekcję `PARAMETRY ROBOTA`. **Kluczowe:** Zmierz linijką i wpisz poprawne wartości dla `rozstawKol_D` (odległość między środkami opon) oraz `srednicaKola`.
   - Upewnij się, że piny w kodzie odpowiadają schematowi (Lewy: PWM=5/Enc=2, Prawy: PWM=6/Enc=3).

2. **Uruchomienie regulatorów PID dla obu kół (podpunkt 4.3.1 PDF)**
   - Wgraj kod. Robot domyślnie stoi (`v=0`).
   - Otwórz Serial Monitor. Wyślij komendę `v0.2` (ustaw prędkość 0.2 m/s) oraz `r1000` (promień "nieskończoność" -> jazda prosto).
   - Dostrój PID (komendy `p`, `i`, `d`) obserwując reakcję obu kół[cite: 823]. Parametry są wspólne dla obu silników.

3. **Jazda po prostej i korekcja (podpunkt 4.3.2 i 4.3.3 PDF)**
   - Wyślij `r1000` (lub inną dużą wartość). Obserwuj w Serial Plotterze, czy `predkoscAktualna_L` i `predkoscAktualna_P` pokrywają się z zadanymi wartościami.
   - Dzięki regulatorowi PID z członem całkującym (`I`), robot powinien sam korygować błędy wynikające z różnic w silnikach (znoszenie powinno być minimalne w porównaniu do sterowania w pętli otwartej).

4.  **Jazda po łuku – kinematyka (podpunkt 4.3.4 i 4.3.5 PDF)**
    - Wpisz w terminalu np. `v0.2` oraz `r0.5` (skręt o promieniu 0.5 metra).
    - Funkcja `ustawJazdePoLuku()` automatycznie przeliczy, że jedno koło musi kręcić się szybciej, a drugie wolniej, zgodnie ze wzorami kinematycznymi.

### Obsługa Terminala (Ćwiczenie 4)
W tym zadaniu komunikacja została rozbudowana o kinematykę:
- `vX.X` – zadaj prędkość liniową w m/s (np. `v0.3`).
- `rX.X` – zadaj promień skrętu w metrach (np. `r0.5`). Duża wartość (np. 1000) oznacza jazdę prosto.
- `sX` – (serwisowe) wymuś konkretne obroty/min na obu silnikach pomijając kinematykę.
- `pX`, `iX`, `dX` – zmiana nastaw PID „na żywo” dla obu silników jednocześnie.

### Serial Plotter (Ćwiczenie 4)
Dane wyświetlane są w kolumnach dla obu silników jednocześnie, co pozwala ocenić synchronizację:
- Format: `0 <Zadana_L> <Aktualna_L> <Zadana_P> <Aktualna_P> <PWM_L> <PWM_P>`.


Gemini może popełniać błędy, także co do ludzi, dlatego dokładnie sprawdzaj odpowiedzi. Twoja

Jeśli coś nie działało i udało Ci się to naprawić, zrób pull request z poprawkami.
