#include <SCServo.h>
#include <vector>

SMS_STS Servo_ST;

// Piny UART do komunikacji z serwami
#define S_RXD 18
#define S_TXD 19

std::vector<int> servo_list;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Servo_ST.pSerial = &Serial1;
  
  delay(2000); // Czas na inicjalizację
  Serial.println("Rozpoczynamy ciagle skanowanie serw...");
}

void loop() {
  servo_list.clear(); // Czyszczenie listy przed nowym skanowaniem

  // Skanowanie dostępnych ID serw (od 0 do 100)
  // Jeśli masz serwa o wyższych ID, możesz zwiększyć ten zakres (maksymalnie do 253)
  for(int i = 0; i <= 10; i++){
    int ID = Servo_ST.Ping(i);
    if(ID != -1){
      servo_list.push_back(ID);
    }
  }

  // Zliczanie i wyświetlanie wyników
  int servo_count = servo_list.size();
  Serial.printf("\n--- Wynik skanowania ---\n");
  Serial.printf("Liczba podlaczonych serw: %d\n", servo_count);

  if(servo_count > 0){
    Serial.print("ID znalezionych serw: ");
    for(int i = 0; i < servo_count; i++){
      Serial.print(servo_list[i]);
      if(i < servo_count - 1){
        Serial.print(", "); // Oddziel przecinkiem, jeśli to nie ostatni element
      }
    }
    Serial.println();
  } else {
    Serial.println("Nie znaleziono zadnych serw.");
  }
  Serial.println("------------------------");

  // Opóźnienie przed kolejnym skanowaniem (np. 3 sekundy)
  delay(1000);
}