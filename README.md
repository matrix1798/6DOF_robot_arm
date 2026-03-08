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
* **Komputer nadrzędny:** Raspberry Pi 5 (lub 4B)
* **Sterownik Real-Time:** Mikrokontroler STM32 (np. NUCLEO-F446RE lub BlackPill F401)
* **Elementy wykonawcze:** Serwomechanizmy magistrali szeregowej (Waveshare/Feetech ST3235 na osie dolne, ST3215 na osie górne)
* **Elektronika dodatkowa:** Własny obwód konwertera UART na Half-Duplex (bufor/bramki logiczne), wydajny zasilacz 12V (min. 10A-15A)
* **Mechanika:** Elementy z druku 3D (PETG/ABS/ASA), łożyska kulkowe poprzeczne, łożysko oporowe w osi bazy

### 2. Warstwa Oprogramowania (Software)
* **Projektowanie 3D (CAD):** SolidWorks lub Autodesk Fusion
* **System Operacyjny (RPi):** Ubuntu Linux 
* **Framework Robotyki:** ROS 2 (np. dystrybucja Humble)
  * **Narzędzia:** MoveIt 2 (planowanie trajektorii), RViz (wizualizacja 3D)
* **Języki programowania:** C++ / Python (ROS), C/C++ (STM32)
* **Środowisko wbudowane:** STM32CubeIDE (z wykorzystaniem bibliotek HAL / LL)

### 3. Warstwa Komunikacyjna (Protokoły)
* **Laptop ↔ RPi:** Wi-Fi (SSH, komunikacja węzłów ROS)
* **RPi ↔ STM32:** UART / wirtualny port COM po USB (przesyłanie wyliczonych punktów trajektorii)
* **STM32 ↔ Serwa:** UART (Single-Wire Half-Duplex) - zadawanie pozycji i odczyt danych z enkoderów

## Harmonogram prac:
1. 2.03-8.03:

Przygotowanie repozytorium, rozpisanie stacku technologicznego oraz zapisanie harmonogramu prac.

2. 9.03-15.03:

Zapoznanie się z programem Fusion 360 oraz analiza wybrangeo hardwearu.

3. 16.03-22.03
4. 23.03-29.03
5. 30.03-5.04
6. 6.04-12.04
7. 13.04-19.04
8. 20.04-26.04
9. 27.04-3.05
10. 4.05-10.05
11. 11.05-17.05
12. 18.05-24.05
13. 25.05-31.05
14. 1.06-7.06
15. 8.06-14.06
16. 15.06-21.06
17. 22.06-28.06
18. 29.06-5.07
19. 6.07-12.07
20. 13.07-19.07

