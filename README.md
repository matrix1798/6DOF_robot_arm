# PDI: Budowa antropomorficznego ramienia robota z implementacją algorytmów interpolacji ruchu

Celem pracy jest zaprojektowanie i budowa modelu fizycznego antropomorficznego ramienia robota (o 6 stopniach swobody) oraz opracowanie systemu sterowania robotem, realizującego ruch z wykorzystaniem interpolacji w przestrzeni złączy robota oraz ruch narzędzia z interpolacją liniową w przestrzeni roboczej.

## Zadania do zrealizowania:
1. Zapoznanie się z literaturą.
2. Zaprojektowanie modelu mechanicznego w oprogramowaniu CAD oraz dobór odpowiednich komponentów wykonawczych.
3. Budowa i montaż fizycznego ramienia robota.
4. Zaprojektowanie systemu sterowania.
5. Implementacja programowa kinematyki prostej i odwrotnej manipulatora.
6. Implementacja programowa interpolacji liniowej i osiowej.
7. Wykonanie testów działania robota

### Literatura:
- John J. Craig Wprowadzenie do robotyki, mechanika i sterowanie
- Honczarenko Jerzy, Roboty przemysłowe
- **Kozłowski Karol, Modelowanie i sterowanie robotów**

## Stos Technologiczny

### 1. Warstwa Sprzętowa (Hardware)
* **Komputer nadrzędny:** Raspberry Pi 5 / PC
* **Elementy wykonawcze:** Serwomechanizmy magistrali szeregowej (Waveshare ST3020)
* **Sterownik servomechanizmów:** Dedyowany serial bus controler - ESP32 Servo Driver Expansion Board
* **Zasilanie:** Zasilacz 12V/25A
* **Mechanika:** Elementy z druku 3D (PETG), łożyska kulkowe poprzeczne, łożysko oporowe w osi bazy

### 2. Warstwa Oprogramowania (Software)
* **Projektowanie 3D (CAD):** Autodesk Fusion
* **System Operacyjny:** Linux
* **Framework Robotyki:** ROS 2 (dystrybucja Humble)
  * **Narzędzia:** MoveIt 2 (planowanie trajektorii), RViz (wizualizacja 3D)
* **Języki programowania:** C++ / Python (ROS), C/C++ (ESP32)

### 3. Warstwa Komunikacyjna (Protokoły)
* **Servo Driver ↔ Serwa:** UART (Single-Wire Half-Duplex) - zadawanie pozycji i odczyt danych z enkoderów





