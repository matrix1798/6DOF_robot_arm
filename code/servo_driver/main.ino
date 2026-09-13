#include <SCServo.h>
#include <vector>

SMS_STS Serwo_ST;
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#define NUM_SERVOS 2

std::vector<int> servo_list;

int pos;
int voltage;
int temper;
int current;

byte id_array[NUM_SERVOS] = {1,2};
int16_t pos_array[NUM_SERVOS] = {0,0};
uint16_t spd_array[NUM_SERVOS];
byte acc_array[NUM_SERVOS];

cnost byte num_chars = 64;
char received_chars[num_chars];
boolean new_data = false;

void setup() {
  //connect config
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serwo_ST.pSerial = &Serial1;
  delay(5000);

  //Scan avaiable servo ID
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

  // disable hardware speed and accleration limits
  for(int i = 0; i < NUM_SERVOS; i++){
    spd_array[i] = 0;
    acc_array[i] = 0;
  }

}

void loop() {
/*
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
*/
  recvMessage();

  if(new_data == true){
    convertData();

    Servo_ST.SyncWritePosEx(id_array, pos_array, spd_array, acc_array, NUM_SERVOS)

    new_data = false;
  }

}

// reading signs from USB port without cpu blocking (no delay)
void recvMessage(){
  static boolean recv_in_progress = false;
  static byte ndx = 0;
  char start_marker = "<"
  char end_marker = ">"
  char rc;

  while(Serial.availabel() > 0 && new_data == false){
    rc = Serial.read()

    if(recv_in_progress = true){
      if (rc != end_marker) {
        received_chars[ndx] = rc;
        ndx++;
        if(ndx >= num_chars) {ndx = ndx = num_chars -1};
      }
      else {
        received_chars[ndx] = '\0';
        recv_in_progress = false;
        ndx = 0;
        new_data = true;
      }
    }
    else if(rc == start_marker){}
      recv_in_progres = true;
    }
  }
}

// Split received_chars
void converData {
  char * stork_indx;

  stork_indx = strtok(received_chars, ',');

  for(int i = 0; i < NUM_SERVOS; i++){
    if(stork_indx != NULL) {
      pos_array[i] = atoi(stork_indx);
      stork_indx = strtok(NULL, ',')
    }
  }
}