from serial import threaded
import time
from serial.tools import list_ports
import string

uart_port = ""
port_log =  ""
global ENCODING
data_gcode = []
class SerialCommunication(serial.threaded.LineReader):
    TERMINATOR = b'\r\n'
    ENCODING = 'utf-8'
    UNICODE_HANDLING = 'replace'

    def connection_made(self, transport):
        """Called when the serial connection is established."""
        super().connection_made(transport)
        print("Connection established")

    def data_received(self, data):
        """Handles raw data received."""
        print("Raw data received:", data.decode(ENCODING))

    def handle_line(self, line):
        """Handles complete lines received."""
        print("Line received:", line)

    def connection_lost(self, exc):
        """Called when the serial connection is lost."""
        if exc:
            print("Connection lost due to an error:", exc)
        else:
            print("Connection closed cleanly")
def read_data():
    with open("data.txt", "r") as f:
        for lines in f.readlines():
            if '(' in lines:
                lines = lines[0:lines.find("(") - 1]
            lines = lines.rstrip("\n")
            data_gcode.append(lines)
    f.close()
def send_data_uart():
    try:
        ports = list_ports.comports()
        for port, desc, hwid in sorted(ports):
            port_log = "{}: {} [{}]".format(port, desc, hwid)
            if port_log.find("CP210x") or port_log.find("CH340x") or port_log.find("PL2303"):
                uart_port = port
        print("Found com: {}", uart_port)
        ser = serial.Serial(uart_port, 115200, timeout=1)
        thread_com = serial.threaded.ReaderThread(ser, SerialCommunication)
        with thread_com as protocol: #Start the ReaderThread
            for lines in data_gcode:
                uart_lines = "{}\n".format(lines).encode('utf-8')
                protocol.transport.write(uart_lines)  # Send data as bytes
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except serial.SerialException as e:
        print("Serial error:", e)
    finally:
        if 'thread_com' in locals() and thread_com.is_alive():
            thread_com.close()
        print("Program terminated")
if _name_ == '_main_':
    read_data()
    send_data_uart()