import time
from serial.tools import list_ports
import serial
from serial import threaded

ENCODING = 'utf-8'

# Error & Alarm Dictionaries
error_codes_to_message = {
    1: "Expected command letter",
    2: "Bad number format",
    3: "Invalid statement",
    4: "Value < 0",
    5: "Setting disabled",
    6: "Value < 3 usec",
    7: "EEPROM read fail. Using defaults",
    8: "Not idle",
    9: "G-code lock",
    10: "Homing not enabled",
    11: "Line overflow",
    12: "Step rate > 30kHz",
    13: "Check Door",
    14: "Line length exceeded",
    15: "Travel exceeded",
    16: "Invalid jog command",
    17: "Setting disabled",
    20: "Unsupported command",
    21: "Modal group violation",
    22: "Undefined feed rate",
    23: "Invalid gcode ID:23",
    24: "Invalid gcode ID:24",
    25: "Invalid gcode ID:25",
    26: "Invalid gcode ID:26",
    27: "Invalid gcode ID:27",
    28: "Invalid gcode ID:28",
    29: "Invalid gcode ID:29",
    30: "Invalid gcode ID:30",
    31: "Invalid gcode ID:31",
    32: "Invalid gcode ID:32",
    33: "Invalid gcode ID:33",
    34: "Invalid gcode ID:34",
    35: "Invalid gcode ID:35",
    36: "Invalid gcode ID:36",
    37: "Invalid gcode ID:37",
    38: "Invalid gcode ID:38"
}

alarm_codes_to_message = {
    1: "Hard limit triggered. Re-homing recommended.",
    2: "Soft limit exceeded. Machine position retained.",
    3: "Reset while in motion. Re-homing recommended.",
    4: "Probe fail (initial state not expected).",
    5: "Probe did not contact within range.",
    6: "Homing fail. Active cycle reset.",
    7: "Homing fail. Safety door opened.",
    8: "Homing fail. Pull off did not clear limit switch.",
    9: "Homing fail. Limit switch not found."
}


class SerialCommunication(serial.threaded.LineReader):
    TERMINATOR = b'\r\n'
    ENCODING = 'utf-8'

    def __init__(self):
        super().__init__()
        self.ok_received = False

    def handle_line(self, line):
        print("Received:", line)
        if line.strip() == "ok":
            self.ok_received = True
        elif line.startswith("error:"):
            error_id = int(line.split(":")[1])
            print("ERROR:", error_codes_to_message.get(error_id, "Unknown error"))
        elif line.startswith("ALARM:"):
            alarm_id = int(line.split(":")[1])
            print("ALARM:", alarm_codes_to_message.get(alarm_id, "Unknown alarm"))

    def wait_for_ok(self, timeout=2):
        start = time.time()
        self.ok_received = False
        while time.time() - start < timeout:
            if self.ok_received:
                return True
            time.sleep(0.05)
        return False


def find_uart_port():
    ports = list_ports.comports()
    for port, desc, hwid in ports:
        print(f"Found port: {port} ({desc})")
        if "ttyACM0" in port or "USB" in port or "ACM" in port:
            return port
    return None


def send_uart_command(protocol, cmd, wait_ok=True, retries=3):
    for attempt in range(retries):
        protocol.ok_received = False
        full_cmd = f"{cmd}\n".encode(ENCODING)
        protocol.transport.write(full_cmd)
        print(f"Sent: {cmd}")

        if wait_ok:
            if protocol.wait_for_ok():
                return True
            else:
                print(f"[Retry {attempt + 1}] No OK received for: {cmd}")
                time.sleep(0.3)
        else:
            return True
    print(f"❌ Failed to send command after {retries} attempts: {cmd}")
    return False


def reset_system(protocol):
    send_uart_command(protocol, "$X")


def run_initialization_sequence(protocol):
    init_cmds = ["$27=1.5", "$X", "$HX", "$HY", "$HA", "$HB", "$27=0.2", "$HZ","$27=1.5"]
    for cmd in init_cmds:
        send_uart_command(protocol, cmd)


def read_gcode_file(filename="gcode.txt"):
    lines = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if '(' in line:
                line = line.split('(')[0].strip()
            if line:
                lines.append(line)
    print(f"📄 Loaded {len(lines)} G-code lines.")
    return lines


def send_gcode_file(protocol, gcode_lines):
    total = len(gcode_lines)
    for idx, line in enumerate(gcode_lines, start=1):
        success = send_uart_command(protocol, line)
        if not success:
            print(f"⚠️ Failed to send line {idx}: {line}")
        else:
            print(f"✅ Sent [{idx}/{total}] {line}")


def start_serial_communication():
    uart_port = find_uart_port()
    if not uart_port:
        print("❌ No UART port found.")
        return None

    try:
        ser = serial.Serial(uart_port, 115200, timeout=1)
        thread_com = serial.threaded.ReaderThread(ser, SerialCommunication)

        thread_com.start()
        protocol = thread_com.connect()[1]

        time.sleep(2)  # Wait for GRBL to boot
        print("📡 Serial connected. Sending $$ for configuration...")
        send_uart_command(protocol, "$$", wait_ok=False)  # GRBL sends settings back
        return protocol

    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        return None


# Example usage (from UI)
if __name__ == '__main__':
    protocol = start_serial_communication()
    if protocol:
        reset_system(protocol)

        # Optional: run init
        # run_initialization_sequence(protocol)

        gcode = read_gcode_file()
        send_gcode_file(protocol, gcode)
