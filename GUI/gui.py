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

        # Create auto-tension button at the top
        self.tension_button = ttk.Button(
            self.main_container,
            text="Auto-Tension All Motors",
            command=self.start_auto_tension
        )
        self.tension_button.pack(pady=(0, 10))

        # Create status frame for tensioning status
        self.status_frame = ttk.LabelFrame(self.main_container, text="Tensioning Status")
        self.status_frame.pack(fill='x', pady=(0, 10))
        
        # Create status widgets for each motor
        self.motor_frames = []
        for i in range(3):
            frame = ttk.Frame(self.status_frame)
            frame.pack(fill='x', padx=5, pady=2)
            
            # Status label
            status_label = ttk.Label(frame, text=f"Motor {i}: Not Started")
            status_label.pack(side=tk.LEFT, padx=5)
            
            # Progress label
            progress_label = ttk.Label(frame, text="Position change: ---")
            progress_label.pack(side=tk.RIGHT, padx=5)
            
            self.motor_frames.append({
                'frame': frame,
                'status': status_label,
                'progress': progress_label
            })

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
        """Create control buttons for each ODrive tab"""
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

        # Create Stop button
        stop_button = ttk.Button(
            button_frame,
            text="Stop",
            command=lambda: self.stop_motor(drive_num)
        )
        stop_button.pack(side=tk.LEFT, padx=10)

    def create_feedback_display(self):
        """Create the feedback display area"""
        # Create frame for feedback
        feedback_frame = ttk.LabelFrame(
            self.main_container, text="System Feedback")
        feedback_frame.pack(fill='both', expand=True, pady=10)

        # Create scrolled text widget
        self.feedback_text = scrolledtext.ScrolledText(
            feedback_frame,
            height=10,
            wrap=tk.WORD
        )
        self.feedback_text.pack(fill='both', expand=True, padx=5, pady=5)

    def start_auto_tension(self):
        """Start the auto-tensioning sequence"""
        if self.ser:
            # Disable the tension button during the process
            self.tension_button.configure(state='disabled')
            
            # Clear previous status
            for i in range(3):
                self.update_motor_status(i, "Waiting...", None)
            
            # Send auto-tension command
            self.ser.write("auto_tension\n".encode())
            
            # Add initial message to feedback
            self.message_queue.put("Starting auto-tensioning sequence...")

    def rotate_cw(self, drive_num):
        """Command the specified drive to rotate clockwise"""
        if self.ser:
            message = f"{drive_num},true\n"
            self.ser.write(message.encode())
            message = f"{drive_num},get_current\n"
            self.ser.write(message.encode())

    def rotate_ccw(self, drive_num):
        """Command the specified drive to rotate counter-clockwise"""
        if self.ser:
            message = f"{drive_num},false\n"
            self.ser.write(message.encode())
            message = f"{drive_num},get_current\n"
            self.ser.write(message.encode())

    def stop_motor(self, drive_num):
        """Stop the specified motor"""
        if self.ser:
            # Send stop command (implementation depends on your Arduino code)
            message = f"{drive_num},stop\n"
            self.ser.write(message.encode())

    def update_motor_status(self, motor_num, message, position_change=None):
        """Update the status display for a specific motor"""
        if 0 <= motor_num < 3:
            motor_frame = self.motor_frames[motor_num]
            motor_frame['status'].configure(text=f"Motor {motor_num}: {message}")
            if position_change is not None:
                motor_frame['progress'].configure(
                    text=f"Position change: {position_change:.6f}")

    def read_serial(self):
        """Read from serial port and add messages to the queue"""
        while self.running:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.message_queue.put(line)
                except (serial.SerialException, UnicodeDecodeError) as e:
                    self.message_queue.put(f"Serial Error: {str(e)}")
                    break

    def update_feedback(self):
        """Process messages from the serial queue and update the UI"""
        while not self.message_queue.empty():
            message = self.message_queue.get()
            
            # Handle different types of messages
            if "Starting tensioning of Motor" in message:
                motor_num = int(message.split("Motor ")[-1])
                self.update_motor_status(motor_num, "Tensioning...")
            
            elif "position change:" in message:
                try:
                    parts = message.split()
                    motor_num = int(parts[1])
                    position_change = float(parts[-1])
                    self.update_motor_status(motor_num, "Tensioning...", position_change)
                except (ValueError, IndexError):
                    pass
            
            elif "Motor" in message and "tensioned" in message:
                try:
                    motor_num = int(message.split()[1])
                    self.update_motor_status(motor_num, "Tensioned")
                except (ValueError, IndexError):
                    pass
            
            elif "Error detected on Motor" in message:
                try:
                    motor_num = int(message.split("Motor ")[-1])
                    self.update_motor_status(motor_num, "Error Detected!")
                except (ValueError, IndexError):
                    pass
            
            elif "All motors tensioned successfully" in message:
                # Re-enable the tension button
                self.tension_button.configure(state='normal')
            
            # Add all messages to feedback display
            self.feedback_text.insert(tk.END, message + '\n')
            self.feedback_text.see(tk.END)

        # Schedule the next update
        self.root.after(100, self.update_feedback)

    def __del__(self):
        """Cleanup when the object is destroyed"""
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.geometry("600x600")
    root.mainloop()