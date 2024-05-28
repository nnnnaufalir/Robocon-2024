String data_send = "", data_in = "", data_receive = "", buffer[10];

// int i, j, k;

int j = 10;
int i = -4;
int k = 1000;

void setup() {
  Serial.begin(57600);

  Serial.println("Comm Ready");
}

void loop() {
  dataSend(String(i), String(j), String(k));

  delay(1);
}
// String dataSend = "";

// void setup() {
//   // Inisialisasi komunikasi serial pada baud rate 115200
//   Serial.begin(57600);
//   while (!Serial) {
//     ;  // Tunggu hingga port serial siap
//   }
//   Serial.println("ESP32_0 Ready");
// }

// void loop() {
//   // Mengirim pesan berkala dari ESP32
//   dataSend = String("path") + "," + String("retry zone") + "," + String("kosong");
//   Serial.println(dataSend);
//   Serial.flush();

//   // Cek apakah ada data yang masuk dari serial
//   if (Serial.available() > 0) {
//     // Baca data yang masuk dari serial
//     String receivedData = Serial.readStringUntil('\n');
//     receivedData.trim();  // Menghapus karakter whitespace di awal/akhir string

//     // Buat string baru dengan data tambahan dan kirim ke serial monitor
//     String data_0 = receivedData;
//   }
//   // Delay antara pengecekan data serial
//   delay(1);
// }

// void parsing() {
// }

// String receive() {

//   return "tes";
// }
