import tkinter as tk
from tkinter import ttk
import serial

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ODrive Control Interface")
        
        # Initialize serial communication
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 11520)  # Adjust port and baud rate as needed
        except serial.SerialException:
            print("Error: Could not open serial port")
            self.ser = None
        
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(pady=10, expand=True)
        
        # Create tabs for each ODrive
        self.tabs = []
        for i in range(3):
            tab = ttk.Frame(self.notebook)
            self.tabs.append(tab)
            self.notebook.add(tab, text=f"ODrive {i}")
            self.create_controls(tab, i)
    
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
        
    def rotate_cw(self, drive_num):
        print(f"Rotating ODrive {drive_num} clockwise")
        if self.ser:
            message = f"{drive_num},true\n"  # Format: drive_num,direction\n
            self.ser.write(message.encode())
        
    def rotate_ccw(self, drive_num):
        print(f"Rotating ODrive {drive_num} counter-clockwise")
        if self.ser:
            message = f"{drive_num},false\n"  # Format: drive_num,direction\n
            self.ser.write(message.encode())
    
    def __del__(self):
        # Close serial port when the application closes
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.geometry("300x200")  # Set initial window size
    root.mainloop()