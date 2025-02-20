import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import threading
import queue


class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ODrive Control Interface")

        # Create queue for thread-safe communication
        self.message_queue = queue.Queue()

        # Initialize serial communication
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        except serial.SerialException:
            print("Error: Could not open serial port")
            self.ser = None

        # Create main container
        self.main_container = ttk.Frame(root)
        self.main_container.pack(expand=True, fill='both', padx=10, pady=10)

        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.main_container)
        self.notebook.pack(pady=10, expand=True)

        # Create tabs for each ODrive
        self.tabs = []
        for i in range(3):
            tab = ttk.Frame(self.notebook)
            self.tabs.append(tab)
            self.notebook.add(tab, text=f"ODrive {i}")
            self.create_controls(tab, i)

        # Create feedback display area
        self.create_feedback_display()

        # Start serial reading thread
        self.running = True
        self.serial_thread = threading.Thread(
            target=self.read_serial, daemon=True)
        self.serial_thread.start()

        # Schedule the first update check
        self.root.after(100, self.update_feedback)

    def create_controls(self, tab, drive_num):
        # Create frame for buttons
        button_frame = ttk.Frame(tab)
        button_frame.pack(pady=20)

        # Create CW button
        cw_button = ttk.Button(
            button_frame,
            text="CW",
            command=lambda: self.rotate_cw(drive_num)
        )
        cw_button.pack(side=tk.LEFT, padx=10)

        # Create CCW button
        ccw_button = ttk.Button(
            button_frame,
            text="CCW",
            command=lambda: self.rotate_ccw(drive_num)
        )
        ccw_button.pack(side=tk.LEFT, padx=10)

        # Create Zero-Impedance button
        zero_imp_button = ttk.Button(
            button_frame,
            text="auto-tensioning",
            command=lambda: self.autotensioning(drive_num)
        )
        zero_imp_button.pack(side=tk.LEFT, padx=10)

    def create_feedback_display(self):
        # Create frame for feedback
        feedback_frame = ttk.LabelFrame(
            self.main_container, text="Serial Feedback")
        feedback_frame.pack(fill='both', expand=True, pady=10)

        # Create scrolled text widget
        self.feedback_text = scrolledtext.ScrolledText(
            feedback_frame,
            height=10,
            wrap=tk.WORD
        )
        self.feedback_text.pack(fill='both', expand=True, padx=5, pady=5)

    def read_serial(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.message_queue.put(line)
            except serial.SerialException as e:
                self.message_queue.put(f"Serial error: {str(e)}")
                break
            except UnicodeDecodeError:
                pass

    def update_feedback(self):
        # Process all available messages
        while not self.message_queue.empty():
            message = self.message_queue.get()
            self.feedback_text.insert(tk.END, message + '\n')
            self.feedback_text.see(tk.END)  # Auto-scroll to bottom

        # Schedule the next update
        self.root.after(100, self.update_feedback)

    def rotate_cw(self, drive_num):
        print(f"Rotating ODrive {drive_num} clockwise")
        if self.ser:
            # Send rotation command
            message = f"{drive_num},true\n"
            self.ser.write(message.encode())

            # Request current values
            # New command to request current
            message = f"{drive_num},get_current\n"
            self.ser.write(message.encode())

    def rotate_ccw(self, drive_num):
        print(f"Rotating ODrive {drive_num} counter-clockwise")
        if self.ser:
            # Send rotation command
            message = f"{drive_num},false\n"
            self.ser.write(message.encode())

            # Request current values
            # New command to request current
            message = f"{drive_num},get_current\n"
            self.ser.write(message.encode())

    # Command for setting an ODrive to zero-impedance mode
    def autotensioning(self, drive_num):
        print(f"Setting ODrive {drive_num} to auto-tensioning mode")
        if self.ser:
            # Send rotation command
            message = f"{drive_num},ten\n"
            self.ser.write(message.encode())


    def __del__(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()


if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.geometry("400x500")  # Increased window size to accommodate feedback
    root.mainloop()
