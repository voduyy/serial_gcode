import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import threading
import time
from serial.tools import list_ports
import serial
from serial import threaded
import queue
import datetime

ENCODING = 'utf-8'
IMG_WIDTH = 640
IMG_HEIGHT = 480

def log_uart(msg):
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    full_msg = f"[{timestamp}] {msg}"
    print(full_msg)  # vẫn in ra terminal
    if hasattr(App, 'uart_log_box') and App.uart_log_box:
        App.uart_log_box.configure(state='normal')
        App.uart_log_box.insert(tk.END, full_msg + "\n")
        App.uart_log_box.see(tk.END)
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
            time.sleep(0.0000001)
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
    for attempt in range(retries):
        protocol.ok_received = False
        msg = f"{cmd}\n"
        protocol.transport.write(msg.encode(ENCODING))
        log_uart(f"➡️ TX: {cmd}")
        if not wait_ok or protocol.wait_for_receive_ok():
            return True
    return False

def reset_system(protocol):
    # send_uart_command(protocol, chr(0x18))
    send_uart_command(protocol, "$X")
    send_uart_command(protocol, "G28")
    send_uart_command(protocol, "F2700")
    done = protocol.get_done_count()

def run_initialization_sequence(protocol, stop_event):
    cmds = ["$X", "$HX", "$HY", "$HZ", "$HA", "$HB"]
    for cmd in cmds:
        success = send_uart_command(protocol, cmd)
    done = protocol.get_done_count()

    # idx = 0
    # while idx < len(cmds):
    #     if stop_event.is_set():
    #         # print("🛑 Stop yêu cầu - dừng gửi")
    #         continue
    #     cmd = cmds[idx]
    #     success = send_uart_command(protocol, cmd)
    #     if success and cmd == "$X" or protocol.get_done_count():
    #         idx +=1

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
            # print("🛑 Dừng giữa batch gửi.")
            break
        if shared_state['sent'] < total_cmds and shared_state['on_flight'] < 36:
            cmd = gcode_lines[shared_state['sent']]
            if is_m_command(cmd):
                print("Sending:", cmd)
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

def send_gcode_file(protocol, gcode_lines, total_cmds, shared_state,
                    queue_lock, receive_done_signal, stop_event):
    send_gcode_package(protocol, gcode_lines, total_cmds, shared_state, package=36, stop_event=stop_event)
    while shared_state['sent'] < total_cmds and not stop_event.is_set():
        receive_done_signal.wait()
        if stop_event.is_set():
            # print("🛑 Stop yêu cầu - dừng gửi")
            break
        with queue_lock:
            send_gcode_package(protocol, gcode_lines, total_cmds, shared_state, package=36, stop_event=stop_event)
            # print(f"📤 Sent: {shared_state['sent']}  📡 On-flight: {shared_state['on_flight']}")
            receive_done_signal.clear()

def receive_gcode_response(protocol, total_cmds, shared_state,
                           queue_lock, receive_done_signal, stop_event):
    while (shared_state['received'] < total_cmds or shared_state['on_flight'] > 0) and not stop_event.is_set():
        done_count = protocol.get_done_count(timeout=1.0)
        if done_count:
            with queue_lock:
                shared_state['received'] += done_count
                shared_state['on_flight'] -= done_count
                print(f"✅ Done: {shared_state['received']}  📉 On-flight: {shared_state['on_flight']}")
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
        ttk.Radiobutton(source_frame, text="Camera", variable=self.source_var, value="camera", command=self.switch_source).pack(side="left")
        ttk.Radiobutton(source_frame, text="Image", variable=self.source_var, value="image", command=self.switch_source).pack(side="left")
        ttk.Button(source_frame, text="📂 Choose Image", command=self.choose_image).pack(side="left", padx=5)
        ttk.Button(source_frame, text="📸 Chụp hình", command=self.capture_frame).pack(side="left", padx=5)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)

        # 🔧 G-code file path + select button
        gcode_frame = ttk.Frame(button_frame)
        gcode_frame.pack(side="left", padx=5)
        self.gcode_entry = ttk.Entry(gcode_frame, width=40, textvariable=self.gcode_path_var, state="readonly")
        self.gcode_entry.pack(side="left")
        ttk.Button(gcode_frame, text="📂", width=3, command=self.choose_gcode_file).pack(side="left", padx=2)

        # Các nút điều khiển
        ttk.Button(button_frame, text="📤 Send", width=12, command=self.do_send_gcode).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🛑 Stop", width=12, command=self.do_stop).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔁 Continue", width=12, command=self.do_continue_gcode).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🏠 Homing", width=12, command=self.do_homing).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔄 Reset", width=12, command=self.do_reset).pack(side="left", padx=5)

        # Lệnh thủ công
        manual_frame = ttk.Frame(self.root)
        manual_frame.pack(pady=5)
        self.manual_entry = ttk.Entry(manual_frame, width=40)
        self.manual_entry.pack(side="left", padx=5)
        self.manual_entry.bind("<Return>", lambda event: self.send_manual_command())
        ttk.Button(manual_frame, text="📨 Gửi lệnh", command=self.send_manual_command).pack(side="left", padx=5)

        # --- UART Log Box ---
        log_frame = ttk.LabelFrame(self.root, text="📜 UART Log Terminal")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.uart_log_box = tk.Text(log_frame, height=15, wrap="word", bg="white", fg="navy", insertbackground="white")
        self.uart_log_box.pack(fill="both", expand=True)
        self.uart_log_box.configure(state='disabled')  # không cho sửa nội dung
        App.uart_log_box = self.uart_log_box  # Để truy cập từ log_uart

    def choose_gcode_file(self):
        filepath = filedialog.askopenfilename(filetypes=[("G-code files", "*.txt *.gcode"), ("All files", "*.*")])
        if filepath:
            self.gcode_file_path = filepath
            self.gcode_path_var.set(filepath)
            # print(f"📁 Đã chọn file G-code: {filepath}")

    def do_send_gcode(self):
        if not self.gcode_file_path:
            messagebox.showerror("Lỗi", "Bạn chưa chọn file G-code.")
            return
        self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
        threading.Thread(target=self.send_gcode_in_background, daemon=True).start()

    def do_continue_gcode(self):
        if self.gcode_file_path:
            # print(f"🔁 Tiếp tục từ dòng {self.shared_state['sent']}")
            self.stop_event.clear()
            threading.Thread(target=self.send_gcode_in_background, daemon=True).start()

    def do_stop(self):
        self.stop_event.set()
        # print("🛑 Dừng gửi.")
        if self.protocol:
            # # send_uart_command(self.protocol, chr(0x18))
            # time.sleep(0.1)
            send_uart_command(self.protocol, "$X")

    def send_gcode_in_background(self):
        gcode_lines = read_gcode_file(self.gcode_file_path)
        total_cmds = len(gcode_lines)
        self.stop_event.clear()
        self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
        # start_time = time.time()
        threading.Thread(target=send_gcode_file,
                         args=(self.protocol, gcode_lines, total_cmds,
                               self.shared_state, self.queue_lock, self.receive_done_signal, self.stop_event),
                         daemon=True).start()
        threading.Thread(target=receive_gcode_response,
                         args=(self.protocol, total_cmds,
                               self.shared_state, self.queue_lock, self.receive_done_signal, self.stop_event),
                         daemon=True).start()
        # end_time = time.time()
        # print(f"Execution time: {end_time-start_time}")

    def send_manual_command(self):
        if self.protocol:
            cmd = self.manual_entry.get().strip().upper()
            self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
            if cmd in ["CTRL X", "CTRL+X"]:
                self.protocol.transport.write(b'\x18')
                log_uart("➡️ TX: Ctrl+X (0x18)")
            else:
                send_uart_command(self.protocol, cmd)
            self.manual_entry.delete(0, tk.END)

    def start_serial(self):
        port = find_uart_port()
        if not port:
            messagebox.showerror("Error", "Không tìm thấy cổng UART")
            return
        ser = serial.Serial(port, 115200, timeout=1)
        thread = serial.threaded.ReaderThread(ser, SerialCommunication)
        thread.start()
        self.protocol = thread.connect()[1]
        # time.sleep(2)
        send_uart_command(self.protocol, "$$", wait_ok=False)

    def start_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Không thể mở camera.")
            return
        self.update_camera()

    def update_camera(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, 1)
                self.last_frame = frame
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb).resize((IMG_WIDTH, IMG_HEIGHT))
                self.display_image(self.left_img_label, img)
                if self.show_mirror:
                    self.display_image(self.right_img_label, img)
        if self.running:
            self.root.after(30, self.update_camera)

    def switch_source(self):
        if self.source_var.get() == "camera":
            if self.cap is None:
                self.cap = cv2.VideoCapture(0)
            self.show_mirror = True
            self.update_camera()
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
            # cmds = ["$X", "$HX", "$HY", "$HZ", "$HA", "$HB"]
            # total_cmds = len(cmds)
            # self.shared_state = {'on_flight': 0, 'sent': 0, 'received': 0}
            threading.Thread(target=run_initialization_sequence, args=(self.protocol, self.stop_event,), daemon=True).start()


    def do_reset(self):
        if self.protocol:
            threading.Thread(target=reset_system, args=(self.protocol,), daemon=True).start()
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