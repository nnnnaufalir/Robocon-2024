void rx_read() {
  if (Serial.available() > 0) {
    // Baca data yang diterima dari Serial
    String data_receive = Serial.readStringUntil('\n');

    // Array untuk menyimpan hasil parsing
    String data_array[MAX_ELEMENTS];
    int array_index = 0;

    // Parsing string menjadi array
    int start_index = 0;
    int comma_index = data_receive.indexOf(',');
    while (comma_index != -1 && array_index < MAX_ELEMENTS - 1) {
      // Ambil substring dari start_index hingga comma_index
      data_array[array_index++] = data_receive.substring(start_index, comma_index);
      // Perbarui start_index untuk mengabaikan koma
      start_index = comma_index + 1;
      // Temukan indeks koma selanjutnya
      comma_index = data_receive.indexOf(',', start_index);
    }
    // Ambil substring terakhir setelah koma terakhir
    data_array[array_index++] = data_receive.substring(start_index);

    Serial.println(data_array[0]);

    // // Cetak array hasil parsing ke Serial Monitor
    // for (int i = 0; i < array_index; i++) {
    //   Serial.print("Data ");
    //   Serial.print(i);
    //   Serial.print(": ");
    //   Serial.println(data_array[i]);
    // }
  }
}