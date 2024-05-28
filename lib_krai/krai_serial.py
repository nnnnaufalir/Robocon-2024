import serial
import time


class SerialCommunicator:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

    def begin(self):
        if not self.ser.is_open:
            self.ser.open()

    def end(self):
        if self.ser.is_open:
            self.ser.close()

    def available(self):
        return self.ser.in_waiting > 0

    def write(self, data, delay_ms):
        time_ms = (delay_ms/1000)
        time.sleep(time_ms)
        data = data + "\n"
        self.ser.write(data.encode('utf-8'))

    def read(self, delay_ms):
        if self.available():
            time_ms = (delay_ms/1000)
            time.sleep(time_ms)
            data = str(self.ser.readline()[:len(self.ser.readline())-1])
            result = data[2:len(data)-3]
            return result
        return None

    def parsing(self, separator, count, delay_ms):
        data = self.read(delay_ms)
        warning = "Data is not completed:" + data
        if data.count(separator) == count:
            data_receive = data.split(separator)
            return data_receive
        return warning

    def flush(self):
        self.ser.flush()

    def flush_input(self):
        self.ser.flushInput()

    def flush_output(self):
        self.ser.flushOutput()
