// Variabel global
long prevTime = 0;
const int interval = 1000;
bool state = false;

void setup() {
  // Inisialisasi komunikasi serial pada baud rate 115200
  Serial.begin(115200);
  while (!Serial) {
    ;  // Tunggu hingga port serial siap
  }
  Serial.println("ESP32_1 Ready");

  // Inisialisasi pin 26 sebagai output
  pinMode(26, OUTPUT);
}

void loop() {
  // Mengirim pesan berkala dari ESP32
  Serial.println("Dari ESP32_1");
  Serial.flush();

  // Cek apakah ada data yang masuk dari serial
  if (Serial.available() > 0) {
    // Baca data yang masuk dari serial
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();  // Menghapus karakter whitespace di awal/akhir string

    // Buat string baru dengan data tambahan dan kirim ke serial monitor
    String data_1 = receivedData + ",data_tiga";
    // Serial.println(data_0);
  }

  // Mengatur waktu dan status LED berkedip
  long currentTime = millis();
  if ((currentTime - prevTime) >= interval) {
    state = !state;
    digitalWrite(26, state);
    prevTime = currentTime;
  } else {
    digitalWrite(26, LOW);
  }

  // Delay antara pengecekan data serial
  delay(1);
}
