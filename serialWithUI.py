import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import threading
import time
import serial
from serial.tools import list_ports
from serial import threaded
import queue
import datetime

ENCODING = 'utf-8'
IMG_WIDTH = 640
IMG_HEIGHT = 480
MAX_LOG_LINES = 500

error_codes_to_message = [
    (1, "Expected command letter", "Expected command letter", "G-code words consist of a letter and a value. Letter was not found."),
    (2, "Bad number format", "Bad number format", "Missing the expected G-code word value or numeric value format is not valid."),
    (3, "Invalid statement", "Invalid statement", "Grbl '$' system command was not recognized or supported."),
    (4, "Value < 0", "Value < 0", "Negative value received for an expected positive value."),
    (5, "Setting disabled", "Setting disabled", "Homing cycle failure. Homing is not enabled via settings."),
    (6, "Value < 3 usec", "Value < 3 usec", "Minimum step pulse time must be greater than 3usec."),
    (7, "EEPROM read fail", "EEPROM read fail. Using defaults", "An EEPROM read failed. Auto-restoring affected EEPROM to default values."),
    (8, "Not idle", "Not idle", "Grbl '$' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job."),
    (9, "G-code lock", "G-code lock", "G-code commands are locked out during alarm or jog state."),
    (10, "Homing not enabled", "Homing not enabled", "Soft limits cannot be enabled without homing also enabled."),
    (11, "Line overflow", "Line overflow", "Max characters per line exceeded. Received command line was not executed."),
    (12, "Step rate > 30kHz", "Step rate > 30kHz", "Grbl '$' setting value cause the step rate to exceed the maximum supported."),
    (13, "Check Door", "Check Door", "Safety door detected as opened and door state initiated."),
    (14, "Line length exceeded", "Line length exceeded", "Build info or startup line exceeded EEPROM line length limit. Line not stored."),
    (15, "Travel exceeded", "Travel exceeded", "Jog target exceeds machine travel. Jog command has been ignored."),
    (16, "Invalid jog command", "Invalid jog command", "Jog command has no '=' or contains prohibited g-code."),
    (17, "Laser mode requires PWM", "Setting disabled", "Laser mode requires PWM output."),
    (20, "Unsupported command", "Unsupported command", "Unsupported or invalid g-code command found in block."),
    (21, "Modal group violation", "Modal group violation", "More than one g-code command from same modal group found in block."),
    (22, "Undefined feed rate", "Undefined feed rate", "Feed rate has not yet been set or is undefined."),
    (23, "Invalid gcode ID:23", "Invalid gcode ID:23", "G-code command in block requires an integer value."),
    (24, "Invalid gcode ID:24", "Invalid gcode ID:24", "More than one g-code command that requires axis words found in block."),
    (25, "Invalid gcode ID:25", "Invalid gcode ID:25", "Repeated g-code word found in block."),
    (26, "Invalid gcode ID:26", "Invalid gcode ID:26", "No axis words found in block for g-code command or current modal state which requires them."),
    (27, "Invalid gcode ID:27", "Invalid gcode ID:27", "Line number value is invalid."),
    (28, "Invalid gcode ID:28", "Invalid gcode ID:28", "G-code command is missing a required value word."),
    (29, "Invalid gcode ID:29", "Invalid gcode ID:29", "G59.x work coordinate systems are not supported."),
    (30, "Invalid gcode ID:30", "Invalid gcode ID:30", "G53 only allowed with G0 and G1 motion modes."),
    (31, "Invalid gcode ID:31", "Invalid gcode ID:31", "Axis words found in block when no command or current modal state uses them."),
    (32, "Invalid gcode ID:32", "Invalid gcode ID:32", "G2 and G3 arcs require at least one in-plane axis word."),
    (33, "Invalid gcode ID:33", "Invalid gcode ID:33", "Motion command target is invalid."),
    (34, "Invalid gcode ID:34", "Invalid gcode ID:34", "Arc radius value is invalid."),
    (35, "Invalid gcode ID:35", "Invalid gcode ID:35", "G2 and G3 arcs require at least one in-plane offset word."),
    (36, "Invalid gcode ID:36", "Invalid gcode ID:36", "Unused value words found in block."),
    (37, "Invalid gcode ID:37", "Invalid gcode ID:37", "G43.1 dynamic tool length offset is not assigned to configured tool length axis."),
    (38, "Invalid gcode ID:38", "Invalid gcode ID:38", "Tool number greater than max supported value.")
]

alarm_codes_to_message = [
    (1, "Hard limit", "Hard limit", "Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended."),
    (2, "Soft limit", "Soft limit", "Soft limit alarm. G-code motion target exceeds machine travel. Machine position retained. Alarm may be safely unlocked."),
    (3, "Abort during cycle", "Abort during cycle", "Reset while in motion. Machine position is likely lost due to sudden halt. Re-homing is highly recommended."),
    (4, "Probe fail", "Probe fail", "Probe is not in the expected initial state before starting probe cycle."),
    (5, "Probe fail", "Probe fail", "Probe did not contact the workpiece within the programmed travel."),
    (6, "Homing fail", "Homing fail", "The active homing cycle was reset."),
    (7, "Homing fail", "Homing fail", "Safety door was opened during homing cycle."),
    (8, "Homing fail", "Homing fail", "Pull off travel failed to clear limit switch. Try increasing pull-off."),
    (9, "Homing fail", "Homing fail", "Could not find limit switch. Try increasing max travel or check wiring.")
]

grbl_settings = [
    ('$0', 'Step pulse time', 'µs', 'Sets time length per step. Minimum 3µs.'),
    ('$1', 'Step idle delay', 'ms', 'Hold delay before disabling steppers. 255 keeps motors enabled.'),
    ('$2', 'Step pulse invert', 'mask', 'Inverts the step signal. Set axis bit to invert (00000ZYX).'),
    ('$3', 'Step direction invert', 'mask', 'Inverts the direction signal. Set axis bit to invert (00000ZYX).'),
    ('$4', 'Invert step enable pin', 'bool', 'Inverts the stepper driver enable pin signal.'),
    ('$5', 'Invert limit pins', 'bool', 'Inverts all limit input pins.'),
    ('$6', 'Invert probe pin', 'bool', 'Inverts the probe input pin signal.'),
    ('$10', 'Status report options', 'mask', 'Alters data included in status reports.'),
    ('$11', 'Junction deviation', 'mm', 'Controls speed through junctions. Lower = slower.'),
    ('$12', 'Arc tolerance', 'mm', 'Sets G2/G3 arc tracing accuracy.'),
    ('$13', 'Report in inches', 'bool', 'Enables inch units in reports.'),
    ('$20', 'Soft limits enable', 'bool', 'Enable soft limits. Requires homing.'),
    ('$21', 'Hard limits enable', 'bool', 'Enable hard limits. Triggers alarm on switch.'),
    ('$22', 'Homing cycle enable', 'bool', 'Enable homing cycle. Requires limit switches.'),
    ('$23', 'Homing direction invert', 'mask', 'Invert homing direction. Bitmask (00000ZYX).'),
    ('$24', 'Homing locate feed rate', 'mm/min', 'Slow rate to locate switch accurately.'),
    ('$25', 'Homing search seek rate', 'mm/min', 'Fast rate to find limit switch.'),
    ('$26', 'Homing debounce delay', 'ms', 'Delay to debounce switch during homing.'),
    ('$27', 'Homing pull-off distance', 'mm', 'Retract after switch trigger. Must clear switch.'),
    ('$30', 'Max spindle speed', 'RPM', 'Spindle speed at 100% PWM duty.'),
    ('$31', 'Min spindle speed', 'RPM', 'Spindle speed at 0.4% PWM duty.'),
    ('$32', 'Laser-mode enable', 'bool', 'Enable laser mode. Avoids halts on spindle changes.'),
    ('$100', 'X steps/mm', 'steps/mm', 'Steps per mm for X-axis.'),
    ('$101', 'Y steps/mm', 'steps/mm', 'Steps per mm for Y-axis.'),
    ('$102', 'Z steps/mm', 'steps/mm', 'Steps per mm for Z-axis.'),
    ('$110', 'X max rate', 'mm/min', 'Maximum movement rate for X-axis.'),
    ('$111', 'Y max rate', 'mm/min', 'Maximum movement rate for Y-axis.'),
    ('$112', 'Z max rate', 'mm/min', 'Maximum movement rate for Z-axis.'),
    ('$120', 'X acceleration', 'mm/sec^2', 'Acceleration for X-axis. Avoid step loss.'),
    ('$121', 'Y acceleration', 'mm/sec^2', 'Acceleration for Y-axis. Avoid step loss.'),
    ('$122', 'Z acceleration', 'mm/sec^2', 'Acceleration for Z-axis. Avoid step loss.'),
    ('$130', 'X max travel', 'mm', 'Max travel distance for X from home.'),
    ('$131', 'Y max travel', 'mm', 'Max travel distance for Y from home.'),
    ('$132', 'Z max travel', 'mm', 'Max travel distance for Z from home.'),
]


def show_error_codes_window(root):
    win = tk.Toplevel(root)
    win.title("📘 GRBL Error Codes")
    win.geometry("900x500")

    tree = ttk.Treeview(win, columns=("type", "code", "short_msg", "old_msg", "description"), show="headings")
    tree.pack(fill="both", expand=True)

    # Đặt tên cột
    tree.heading("type", text="Type")
    tree.heading("code", text="Code")
    tree.heading("short_msg", text="Short Message")
    tree.heading("old_msg", text="Old Message")
    tree.heading("description", text="Description")

    # Cài đặt chiều rộng các cột
    tree.column("type", width=80, anchor="center")
    tree.column("code", width=50, anchor="center")
    tree.column("short_msg", width=180)
    tree.column("old_msg", width=180)
    tree.column("description", width=400)

    # Thêm dữ liệu ERROR
    for code, short_msg, old_msg, desc in error_codes_to_message:
        tree.insert("", "end", values=("ERROR", code, short_msg, old_msg, desc))


def show_alarm_codes_window(root):
    win = tk.Toplevel(root)
    win.title("📘 GRBL Alarm Codes")
    win.geometry("900x500")

    tree = ttk.Treeview(win, columns=("type", "code", "short_msg", "old_msg", "description"), show="headings")
    tree.pack(fill="both", expand=True)

    # Đặt tên cột
    tree.heading("type", text="Type")
    tree.heading("code", text="Code")
    tree.heading("short_msg", text="Short Message")
    tree.heading("old_msg", text="Old Message")
    tree.heading("description", text="Description")

    # Cài đặt chiều rộng các cột
    tree.column("type", width=80, anchor="center")
    tree.column("code", width=50, anchor="center")
    tree.column("short_msg", width=180)
    tree.column("old_msg", width=180)
    tree.column("description", width=400)
    for code, short_msg, old_msg, desc in alarm_codes_to_message:
        tree.insert("", "end", values=("ALARM", code, short_msg, old_msg, desc))

def show_setting_codes_window(root):
    win = tk.Toplevel(root)
    win.title("⚙️ GRBL Setting Codes")
    win.geometry("800x500")
    tree = ttk.Treeview(win, columns=("code", "name", "unit", "desc"), show="headings")
    tree.pack(fill="both", expand=True)
    tree.heading("code", text="$-Code")
    tree.heading("name", text="Setting")
    tree.heading("unit", text="Unit")
    tree.heading("desc", text="Description")
    tree.column("code", width=60)
    tree.column("name", width=200)
    tree.column("unit", width=80)
    tree.column("desc", width=440)
    for row in grbl_settings:
        tree.insert("", "end", values=row)

def log_uart(msg):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    full_msg = f"[{timestamp}] {msg}"
    print(full_msg)
    if hasattr(App, 'uart_log_box') and App.uart_log_box:
        App.uart_log_box.configure(state='normal')
        App.uart_log_box.insert(tk.END, full_msg + "\n")
        App.uart_log_box.see(tk.END)

        # if int(App.uart_log_box.index('end-1c').split('.')[0]) > MAX_LOG_LINES:
        #     App.uart_log_box.delete('1.0', '2.0')

        App.uart_log_box.configure(state='disabled')

class SerialCommunication(serial.threaded.LineReader):
    TERMINATOR = b'\r\n'
    def __init__(self):
        super().__init__()
        self.ok_received = False
        self.done_queue = queue.Queue()
        self.ok_lock = threading.Lock()

    def handle_line(self, line):
        log_uart(f"⬅️ RX: {line.strip()}")
        if line.strip() == "ok":
            with self.ok_lock:
                self.ok_received = True
        elif line.strip() == "done":
            self.done_queue.put(1)

    def wait_for_receive_ok(self, timeout=2):
        start = time.time()
        with self.ok_lock:
            self.ok_received = False
        while time.time() - start < timeout:
            with self.ok_lock:
                if self.ok_received:
                    return True
            time.sleep(0.001)
        return False

    def get_done_count(self, max_count=36, timeout=1.0):
        count = 0
        end_time = time.time() + timeout
        while count < max_count:
            remaining = end_time - time.time()
            if remaining <= 0:
                break
            try:
                self.done_queue.get(timeout=remaining)
                count += 1
            except queue.Empty:
                break
        return count

def find_uart_port():
    ports = list_ports.comports()
    for port, desc, hwid in ports:
        if "ttyACM" in port or "USB" in port or "ACM" in port or "COM" in port:
            return port
    return None

def send_uart_command(protocol, cmd, wait_ok=True, retries=3):
    for _ in range(retries):
        protocol.ok_received = False
        msg = f"{cmd}\n"
        protocol.transport.write(msg.encode(ENCODING))
        log_uart(f"➡️ TX: {cmd}")
        if not wait_ok or protocol.wait_for_receive_ok():
            return True
    return False

def reset_system(protocol):
    send_uart_command(protocol, "$X")
    send_uart_command(protocol, "G28")
    protocol.get_done_count()

def run_initialization_sequence(protocol, stop_event):
    cmds = ["$X", "$HX", "$HY", "$HZ", "$HA", "$HB"]
    for cmd in cmds:
        if stop_event.is_set(): return
        send_uart_command(protocol, cmd)
    protocol.get_done_count()

def read_gcode_file(filename):
    lines = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if '(' in line:
                line = line.split('(')[0].strip()
            if line:
                lines.append(line)
    return lines

def is_m_command(cmd):
    return cmd.startswith("M")

def is_motion_command(cmd):
    return cmd.startswith("G0") or cmd.startswith("G1")

def send_gcode_package(protocol, gcode_lines, total_cmds, shared_state, package, stop_event):
    size = min(36 - shared_state['on_flight'], package)
    for _ in range(size):
        if stop_event.is_set():
            break
        if shared_state['sent'] < total_cmds and shared_state['on_flight'] < 36:
            cmd = gcode_lines[shared_state['sent']]
            if is_m_command(cmd):
                protocol.transport.write(f"{cmd}\n".encode(ENCODING))
                shared_state['sent'] += 1
                shared_state['received'] += 1
            else:
                success = send_uart_command(protocol, cmd)
                if success and is_motion_command(cmd):
                    shared_state['sent'] += 1
                    shared_state['on_flight'] += 1
                elif success:
                    shared_state['sent'] += 1
                    shared_state['received'] += 1

def send_gcode_file(protocol, gcode_lines, total_cmds, shared_state, queue_lock, receive_done_signal, stop_event):
    send_gcode_package(protocol, gcode_lines, total_cmds, shared_state, 36, stop_event)
    while shared_state['sent'] < total_cmds and not stop_event.is_set():
        receive_done_signal.wait()
        if stop_event.is_set(): break
        with queue_lock:
            send_gcode_package(protocol, gcode_lines, total_cmds, shared_state, 36, stop_event)
            receive_done_signal.clear()

def receive_gcode_response(protocol, total_cmds, shared_state, queue_lock, receive_done_signal, stop_event):
    while (shared_state['received'] < total_cmds or shared_state['on_flight'] > 0) and not stop_event.is_set():
        done_count = protocol.get_done_count(timeout=1.0)
        if done_count:
            with queue_lock:
                shared_state['received'] += done_count
                shared_state['on_flight'] -= done_count
                receive_done_signal.set()

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("📟 G-code Control Panel")
        self.protocol = None
        self.cap = None
        self.running = True
        self.last_frame = None
        self.show_mirror = True
        self.command_queue = queue.Queue(maxsize=36)
        self.queue_lock = threading.Lock()
        self.receive_done_signal = threading.Event()
        self.stop_event = threading.Event()
        self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
        self.gcode_file_path = None
        self.gcode_path_var = tk.StringVar(value="Chưa chọn file G-code")

        self.build_gui()
        self.start_serial()
        self.start_camera()

    def build_gui(self):
        img_frame = ttk.Frame(self.root)
        img_frame.pack(pady=10)

        self.left_img_label = tk.Label(img_frame)
        self.left_img_label.pack(side="left", padx=10)

        self.right_img_label = tk.Label(img_frame)
        self.right_img_label.pack(side="left", padx=10)

        source_frame = ttk.Frame(self.root)
        source_frame.pack(pady=5)
        self.source_var = tk.StringVar(value="camera")
        ttk.Label(source_frame, text="Source:").pack(side="left")
        ttk.Radiobutton(source_frame, text="Camera Laptop", variable=self.source_var, value="in_camera", command=self.switch_source).pack(side="left")
        ttk.Radiobutton(source_frame, text="Camera ngoài", variable=self.source_var, value="ex_camera", command=self.switch_source).pack(side="left")
        ttk.Button(source_frame, text="📂 Chọn ảnh", command=self.choose_image).pack(side="left", padx=5)
        ttk.Button(source_frame, text="📸 Chụp hình", command=self.capture_frame).pack(side="left", padx=5)
        ttk.Button(source_frame, text="Xử lý ảnh", command=self.image_processing).pack(side="left", padx=5)
        ttk.Button(source_frame, text="Sinh gcode", command=self.generate_gcode).pack(side="left", padx=5)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)

        gcode_frame = ttk.Frame(button_frame)
        gcode_frame.pack(side="left", padx=5)
        self.gcode_entry = ttk.Entry(gcode_frame, width=40, textvariable=self.gcode_path_var, state="readonly")
        self.gcode_entry.pack(side="left")
        ttk.Button(gcode_frame, text="📂", width=3, command=self.choose_gcode_file).pack(side="left", padx=2)

        ttk.Button(button_frame, text="📤 Gửi gcode", width=12, command=self.do_send_gcode).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🛑 Dừng", width=12, command=self.do_stop).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔁 Tiếp tục", width=12, command=self.do_continue_gcode).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🏠 Homing", width=12, command=lambda: threading.Thread(target=self.do_homing, daemon=True).start()).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔄 Reset", width=8, command=lambda: threading.Thread(target=self.do_reset, daemon=True).start()).pack(side="left", padx=5)
        info_frame = ttk.Frame(self.root)
        info_frame.pack(pady=5)
        code_button_frame = ttk.Frame(self.root)
        code_button_frame.pack(anchor="c", padx=10, pady=5)

        ttk.Button(code_button_frame, text="📘 Error Codes", width=14,
                   command=lambda: show_error_codes_window(self.root)).pack(side="left", padx=5)
        ttk.Button(code_button_frame, text="⚙️ Setting Codes", width=16,
                   command=lambda: show_setting_codes_window(self.root)).pack(side="left", padx=5)
        ttk.Button(code_button_frame, text="🚨 Alarm Codes", width=14,
                   command=lambda: show_alarm_codes_window(self.root)).pack(side="left", padx=5)

        manual_frame = ttk.Frame(self.root)
        manual_frame.pack(pady=5)
        self.manual_entry = ttk.Entry(manual_frame, width=40)
        self.manual_entry.pack(side="left", padx=5)
        self.manual_entry.bind("<Return>", lambda event: self.send_manual_command())
        ttk.Button(manual_frame, text="📨 Gửi lệnh", command=self.send_manual_command).pack(side="left", padx=5)

        log_frame = ttk.LabelFrame(self.root, text="📜 UART Log Terminal")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        self.uart_log_box = tk.Text(log_frame, height=15, wrap="word", bg="white", fg="navy", insertbackground="white")
        self.uart_log_box.pack(fill="both", expand=True)
        self.uart_log_box.configure(state='disabled')
        App.uart_log_box = self.uart_log_box

    def image_processing(self):
        return

    def generate_gcode(self):
        return

    def choose_gcode_file(self):
        filepath = filedialog.askopenfilename(filetypes=[("G-code files", "*.txt *.gcode"), ("All files", "*.*")])
        if filepath:
            self.gcode_file_path = filepath
            self.gcode_path_var.set(filepath)

    def do_send_gcode(self):
        if not self.gcode_file_path:
            messagebox.showerror("Lỗi", "Bạn chưa chọn file G-code.")
            return
        self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
        threading.Thread(target=self.send_gcode_in_background, daemon=True).start()

    def do_continue_gcode(self):
        if self.gcode_file_path:
            self.stop_event.clear()
            threading.Thread(target=self.send_gcode_in_background, daemon=True).start()

    def do_stop(self):
        self.stop_event.set()
        if self.protocol:
            send_uart_command(self.protocol, "$X")

    def send_gcode_in_background(self):
        gcode_lines = read_gcode_file(self.gcode_file_path)
        total_cmds = len(gcode_lines)
        self.stop_event.clear()
        self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
        threading.Thread(target=send_gcode_file,
                         args=(self.protocol, gcode_lines, total_cmds, self.shared_state, self.queue_lock, self.receive_done_signal, self.stop_event),
                         daemon=True).start()
        threading.Thread(target=receive_gcode_response,
                         args=(self.protocol, total_cmds, self.shared_state, self.queue_lock, self.receive_done_signal, self.stop_event),
                         daemon=True).start()

    def send_manual_command(self):
        cmd = self.manual_entry.get().strip().upper()
        self.manual_entry.delete(0, tk.END)
        threading.Thread(target=self._manual_cmd_thread, args=(cmd,), daemon=True).start()

    def _manual_cmd_thread(self, cmd):
        if self.protocol:
            self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
            if cmd in ["CTRL X", "CTRL+X"]:
                self.protocol.transport.write(b'\x18')
                log_uart("➡️TX: Ctrl+X (0x18)")
            else:
                send_uart_command(self.protocol, cmd)

    def start_serial(self):
        port = find_uart_port()
        if not port:
            messagebox.showerror("Error", "Không tìm thấy cổng UART")
            return
        ser = serial.Serial(port, 115200, timeout=1)
        thread = serial.threaded.ReaderThread(ser, SerialCommunication)
        thread.start()
        self.protocol = thread.connect()[1]
        send_uart_command(self.protocol, "$$", wait_ok=False)

    def start_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Không thể mở camera.")
            return
        threading.Thread(target=self.capture_loop, daemon=True).start()

    def capture_loop(self):
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, 1)
                self.last_frame = frame
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb).resize((IMG_WIDTH, IMG_HEIGHT))
                self.root.after(0, lambda: self.display_image(self.left_img_label, img))
                if self.show_mirror:
                    self.root.after(0, lambda: self.display_image(self.right_img_label, img))
            time.sleep(0.03)

    def switch_source(self):
        if self.source_var.get() == "in_camera":
            if self.cap is None:
                self.start_camera()
            self.show_mirror = True
        else:
            if self.cap:
                self.cap.release()
                self.cap = None
            self.show_mirror = False

    def choose_image(self):
        filepath = filedialog.askopenfilename()
        if not filepath:
            return
        img = Image.open(filepath).resize((IMG_WIDTH, IMG_HEIGHT))
        self.display_image(self.right_img_label, img)
        self.show_mirror = False

    def capture_frame(self):
        if self.last_frame is not None:
            img = Image.fromarray(cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2RGB)).resize((IMG_WIDTH, IMG_HEIGHT))
            self.display_image(self.right_img_label, img)
            self.show_mirror = False

    def display_image(self, label, img):
        imgtk = ImageTk.PhotoImage(img)
        label.imgtk = imgtk
        label.configure(image=imgtk)

    def do_homing(self):
        if self.protocol:
            run_initialization_sequence(self.protocol, self.stop_event)

    def do_reset(self):
        if self.protocol:
            reset_system(self.protocol)
        self.shared_state['on_flight'] = 0

    def on_close(self):
        self.running = False
        if self.cap:
            self.cap.release()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

#hihi