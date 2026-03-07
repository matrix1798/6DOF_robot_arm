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

## Stack technologiczny:
1. Harweare:
- Komputer nadrzędny: Raspberry Pi 5 (lub 4B) – do obsługi ROS, obliczeń macierzowych i komunikacji ze światem.
- Sterownik Real-Time: Mikrokontroler STM32 (np. NUCLEO-F446RE lub tańszy BlackPill na F401) – do obsługi protokołu serw i rygorystycznej interpolacji czasowej.
- Elementy wykonawcze: Serwomechanizmy Serial Bus (Waveshare/Feetech ST3235 w dolnych osiach, ST3215 w górnych).
- Elektronika dodatkowa: Moduł konwertera UART na Half-Duplex (lub własny obwód na bramkach logicznych) oraz wydajny zasilacz (np. 12V, min. 10A-15A).
- Mechanika: Elementy z druku 3D (zalecany PETG lub ABS/ASA), łożyska kulkowe poprzeczne oraz łożysko oporowe w podstawie.
2. Softwaer:
3. Komunikacja:

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

