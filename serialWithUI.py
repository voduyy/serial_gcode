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

ENCODING = 'utf-8'
IMG_WIDTH = 640
IMG_HEIGHT = 480


class SerialCommunication(serial.threaded.LineReader):
    TERMINATOR = b'\r\n'

    def __init__(self):
        super().__init__()
        self.ok_received = False
        self.ok_process = False
        self.count = 0

    def handle_line(self, line):
        print("Received:", line)
        if line.strip() == "ok":
            self.count += 1
            if self.count == 1:
                self.ok_received = True
            elif self.count > 1:
                self.ok_process = True

    def wait_for_receive_ok(self, timeout=2):
        start = time.time()
        self.ok_received = False
        self.count = 0
        while time.time() - start < timeout:
            if self.ok_received:
                return True
        return False

    def wait_for_process_ok(self, timeout=2):
        start = time.time()
        if not self.ok_received: return False
        self.ok_process = False
        self.count = 0
        while time.time() - start < timeout:
            if self.ok_process:
                return True
        return False

def find_uart_port():
    ports = list_ports.comports()
    for port, desc, hwid in ports:
        if "ttyACM0" in port or "USB" in port or "ACM" in port or "COM" in port:
            return port
    return None

def send_uart_command(protocol, cmd, wait_ok=True, retries=3):
    for attempt in range(retries):
        protocol.ok_received = False
        protocol.transport.write(f"{cmd}\n".encode(ENCODING))
        print("Sent:", cmd)
        if not wait_ok or protocol.wait_for_receive_ok():
            return True
    return False

def receive_uart_signal(protocol, retries=3):
    for attempt in range(retries):
        protocol.ok_process = False
        if protocol.wait_for_receive_ok() and protocol.wait_for_process_ok():
            return True
    return False

def reset_system(protocol):
    send_uart_command(protocol, "$X")

def run_initialization_sequence(protocol):
    cmds = ["$X", "$HX", "$HY", "$HA", "$HB", "$27=0.2", "$HZ"]
    for cmd in cmds:
        send_uart_command(protocol, cmd)

def read_gcode_file(filename="6_gcode.txt"):
    lines = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if '(' in line:
                line = line.split('(')[0].strip()
            if line:
                lines.append(line)
    return lines

def send_gcode_file(protocol, gcode_lines, gcode_queue, total_cmds, shared_state, queue_lock):
    while shared_state['sent'] < total_cmds:
        if not gcode_queue.full():
            with queue_lock:
                line = gcode_lines[shared_state['sent']]
                success = send_uart_command(protocol, line)
                if success:
                    print(f"✅ Sent [{shared_state['sent'] + 1}/{total_cmds}]: {line}")
                    gcode_queue.put((shared_state['sent'], line))
                    shared_state['sent'] += 1
                else:
                    print(f"❌ Failed to send [{shared_state['sent'] + 1}/{total_cmds}]: {line}")
        else:
            time.sleep(0.1)  # tránh chiếm CPU

def receive_gcode_response(protocol, gcode_lines, gcode_queue, total_cmds, shared_state, queue_lock):
    while shared_state['received'] < total_cmds:
        if not gcode_queue.full():
            time.sleep(0.1)
            continue

        with queue_lock:
            success = receive_uart_signal(protocol)
            if success:
                shared_state['received'] += 1
                line = gcode_queue.get()
                print(f"✅ Done [{shared_state['received']}/{total_cmds}]: {line}")
            else:
                print("❌ No done signal from STM32")

class App:

    def __init__(self, root):
        self.root = root
        self.root.title("📟 G-code Control Panel")
        self.protocol = None
        self.cap = None
        self.running = True
        self.last_frame = None
        self.show_mirror = True
        #Process queue commands
        self.command_queue = queue.Queue(maxsize=16)
        self.queue_lock = threading.Lock()
        self.receive_done_signal = threading.Event()

        self.build_gui()
        self.start_serial()
        self.start_camera()

    def build_gui(self):
        # Frame cho 2 hình ảnh
        img_frame = ttk.Frame(self.root)
        img_frame.pack(pady=10)

        self.left_img_label = tk.Label(img_frame)
        self.left_img_label.pack(side="left", padx=10)

        self.right_img_label = tk.Label(img_frame)
        self.right_img_label.pack(side="left", padx=10)

        # Frame chọn nguồn
        source_frame = ttk.Frame(self.root)
        source_frame.pack(pady=5)

        self.source_var = tk.StringVar(value="camera")
        ttk.Label(source_frame, text="Source:").pack(side="left")
        ttk.Radiobutton(source_frame, text="Camera", variable=self.source_var, value="camera", command=self.switch_source).pack(side="left")
        ttk.Radiobutton(source_frame, text="Image", variable=self.source_var, value="image", command=self.switch_source).pack(side="left")
        ttk.Button(source_frame, text="📂 Choose Image", command=self.choose_image).pack(side="left", padx=5)
        ttk.Button(source_frame, text="📸 Chụp hình", command=self.capture_frame).pack(side="left", padx=5)

        # Frame nút chức năng
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="🏠 Homing", width=14, command=self.do_homing).pack(side="left", padx=5)
        ttk.Button(button_frame, text="🔄 Reset", width=14, command=self.do_reset).pack(side="left", padx=5)
        ttk.Button(button_frame, text="📤 Send G-code", width=14, command=self.do_send_gcode).pack(side="left", padx=5)

    def start_serial(self):
        port = find_uart_port()
        if not port:
            messagebox.showerror("Error", "No UART port found!")
            return
        ser = serial.Serial(port, 115200, timeout=1)
        thread = serial.threaded.ReaderThread(ser, SerialCommunication)
        thread.start()
        self.protocol = thread.connect()[1]
        time.sleep(2)
        send_uart_command(self.protocol, "$$", wait_ok=False)

    def start_camera(self):
        # Automatically initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Unable to access the camera.")
            return
        self.update_camera()

    def switch_source(self):
        if self.source_var.get() == "camera":
            if self.cap is None:
                self.cap = cv2.VideoCapture(0)
            self.show_mirror = True  # Bật lại mirror khi chuyển về camera
            self.update_camera()
        else:
            if self.cap:
                self.cap.release()
                self.cap = None
            self.show_mirror = False  # Không mirror nếu dùng ảnh

    def choose_image(self):
        filepath = filedialog.askopenfilename()
        if not filepath:
            return
        img = Image.open(filepath).resize((IMG_WIDTH, IMG_HEIGHT))
        self.display_image(self.right_img_label, img)
        self.show_mirror = False

    def capture_frame(self):
        if self.last_frame is not None:
            # Chuyển đổi frame sang RGB và resize
            img = Image.fromarray(cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2RGB)).resize((IMG_WIDTH, IMG_HEIGHT))
            # Hiển thị ảnh lên frame phải
            self.display_image(self.right_img_label, img)
            self.show_mirror = False
        else:
            print("Không có ảnh để chụp!")

    def update_camera(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.last_frame = frame
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb).resize((IMG_WIDTH, IMG_HEIGHT))

                self.display_image(self.left_img_label, img)

                if self.show_mirror:
                    self.display_image(self.right_img_label, img)

        if self.running:
            self.root.after(30, self.update_camera)

    def display_image(self, label, img):
        imgtk = ImageTk.PhotoImage(img)
        label.imgtk = imgtk
        label.configure(image=imgtk)

    def do_homing(self):
        if self.protocol:
            threading.Thread(target=run_initialization_sequence, args=(self.protocol,), daemon=True).start()

    def do_reset(self):
        if self.protocol:
            threading.Thread(target=reset_system, args=(self.protocol,), daemon=True).start()

    def do_send_gcode(self):
        if self.protocol:
            gcode_lines = read_gcode_file()
            total_cmds = len(gcode_lines)
            self.command_queue = queue.Queue(maxsize=16)
            shared_state = {'idx': 0, 'sent': 0, 'received': 0}
            #Thread xử lý việc gửi lệnh
            thread2 = threading.Thread(target=send_gcode_file, args=(self.protocol, gcode_lines, self.command_queue, total_cmds, shared_state, self.queue_lock), daemon=True).start()
            #Thread xử lý việc nhận lệnh
            thread3 = threading.Thread(target=receive_gcode_response, args=(self.protocol, gcode_lines, self.command_queue, total_cmds, shared_state, self.queue_lock), daemon=True).start()


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