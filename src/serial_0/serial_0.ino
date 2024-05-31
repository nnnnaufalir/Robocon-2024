String data_send = "", data_in = "", data_receive = "", buffer[10];

#define MAX_ELEMENTS 10

int i = 100;
bool j = true;
String k = "oke";


void setup() {
  Serial.begin(57600);
  Serial.println("Comm Ready");
}

void loop() {
  // dataSend(String(i), String(j), String(k));
  rx_read();
  delay(1);
}
