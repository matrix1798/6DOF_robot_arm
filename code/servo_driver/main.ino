#include <SCServo.h>
#include <vector>

SMS_STS Serwo_ST;
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

std::vector<int> servo_list;

int pos;
int voltage;
int temper;
int current;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serwo_ST.pSerial = &Serial1;
  delay(5000);

  Serial.println("\nZaczynamy skanowanie:");

  for(int i = 0; i <= 100; i++){
    int ID = Serwo_ST.Ping(i);
    if(ID != -1){
      Serial.printf("Znaleziono serwo o ID: %d\n", ID);
      servo_list.push_back(ID);
      delay(200);
    }
  }

  delay(1000);

  Serial.println("Dostepne ID serw: ");
  for (int num: servo_list){
    Serial.println(num);
  }

}

void loop() {
  if(Serwo_ST.FeedBack(1)!=-1){
    pos = Serwo_ST.ReadPos(1);
    Serial.printf("Pozycja dla 1: %d\n", pos);
    delay(3000);
  }
  if(Serwo_ST.FeedBack(2)!=-1){
    pos = Serwo_ST.ReadPos(2);
    Serial.printf("Pozycja dla 2: %d\n", pos);
    delay(3000);
  }

  Serwo_ST.RegWritePosEx(1, 500, 1000);
  Serwo_ST.RegWritePosEx(2, 500, 1000);
  Serwo_ST.RegWriteAction();
  delay(3000);

  if(Serwo_ST.FeedBack(1)!=-1){
    pos = Serwo_ST.ReadPos(1);
    Serial.printf("Pozycja dla 1: %d\n", pos);
    delay(3000);
  }
  if(Serwo_ST.FeedBack(2)!=-1){
    pos = Serwo_ST.ReadPos(2);
    Serial.printf("Pozycja dla 2: %d\n", pos);
    delay(3000);
  }

  Serwo_ST.RegWritePosEx(1, 0, 1000);
  Serwo_ST.RegWritePosEx(2, 0, 1000);
  Serwo_ST.RegWriteAction();
  delay(3000);

}
