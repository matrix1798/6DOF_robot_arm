#include <SCServo.h>
#include <vector>

SMS_STS Servo_ST;
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#define NUM_SERVOS 6


std::vector<int> servo_list;
static unsigned long lastSendTime = 0;

byte id_array[NUM_SERVOS] = {1,2,3,4,5,6};
int16_t pos_array[NUM_SERVOS] = {0,0,0,0,0,0};
uint16_t spd_array[NUM_SERVOS];
byte acc_array[NUM_SERVOS];

//feedback data
int pos;
int voltage;
int temper;
int current;
int16_t pos_feed_array[NUM_SERVOS];
int16_t voltage_feed_array[NUM_SERVOS];
int16_t temper_feed_array[NUM_SERVOS];
int16_t current_feed_array[NUM_SERVOS];

// variable to recv data
const byte num_chars = 64;
char received_chars[num_chars];
boolean new_data = false;

void setup() {
  //connect config
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Servo_ST.pSerial = &Serial1;
  delay(5000);

  //Scan avaiable servo ID
  Serial.println("\nZaczynamy skanowanie:");
  for(int i = 0; i <= 100; i++){
    int ID = Servo_ST.Ping(i);
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

  // read all data from servos
  for (int i = 0; i < 6 ; i++){
    int id = id_array[i];
    if(Servo_ST.FeedBack(id)!=-1){
      pos_feed_array[i] = Servo_ST.ReadPos(id);
      voltage_feed_array[i] = Servo_ST.ReadVoltage(id);
      temper_feed_array[i] = Servo_ST.ReadTemper(id);
      current_feed_array[i] = Servo_ST.ReadCurrent(id);
    }
  }

  if ((millis() - lastSendTime) > 100){
    sendMessage();
    lastSendTime = millis();
  }
  recvMessage();
  //Serial.println("Czekam na dane: ");
  if(new_data == true){
    convertData();

    Serial.print("Odczytane pozycje: ");
    Serial.printf("%d, %d\n",pos_array[0],pos_array[1]);
    Servo_ST.SyncWritePosEx(id_array, NUM_SERVOS,pos_array, spd_array, acc_array);
    
    new_data = false;
  }

}

// reading signs from USB port without cpu blocking (no delay)
void recvMessage() {
  static boolean recv_in_progress = false;
  static byte ndx = 0;
  char start_marker = '<';
  char end_marker = '>';
  char rc;

  while(Serial.available() > 0 && new_data == false) {
    rc = Serial.read();

    if(recv_in_progress == true) {
      if (rc != end_marker) {
        received_chars[ndx] = rc;
        ndx++;
        if(ndx >= num_chars) { ndx = num_chars -1; }
      }
      else {
        received_chars[ndx] = '\0'; // Zakończenie stringa
        recv_in_progress = false;
        ndx = 0;
        new_data = true;
      }
    }
    else if(rc == start_marker) {
      recv_in_progress = true;
    }
  }
}

// send feedback data from all servos
void sendMessage() {

  char mesg_part[100] = "";
  char message[600] = "<";
  for (int i = 0; i < NUM_SERVOS; i++){
    int id = id_array[i];
    sprintf(mesg_part, "%d, %d, %d, %d, %d; ", id, pos_feed_array[i],temper_feed_array[i], voltage_feed_array[i],  current_feed_array[i]);
    strcat(message, mesg_part);
  }

  strcat(message,">");

  Serial.println(message);

}

// Split received_chars
void convertData() {
  char * strtok_indx;

  strtok_indx = strtok(received_chars, ",");

  for(int i = 0; i < NUM_SERVOS; i++){
    if(strtok_indx != NULL) {
      pos_array[i] = atoi(strtok_indx);
      strtok_indx = strtok(NULL, ",");
    }
  }
}