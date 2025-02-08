import serial
import json
from datetime import datetime
from rest_framework.response import Response
from rest_framework.decorators import api_view

SERIAL_PORT = "COM8"  # Update with your actual port
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[{datetime.now()}] ✅ Serial port {SERIAL_PORT} opened successfully!")
except Exception as e:
    print(f"[{datetime.now()}] ❌ Error opening serial port: {e}")
    ser = None


@api_view(["POST"])
def move_arm(request):
    """Receive slider values and send to robotic finger."""
    if not ser:
        print(f"[{datetime.now()}] ❌ Serial port not available!")
        return Response({"error": "Serial port not available"}, status=500)

    data = json.loads(request.body)

    # Extract angles
    knuckle1 = data.get("knuckle1", 90)
    knuckle2 = data.get("knuckle2", 90)
    knuckle3 = data.get("knuckle3", 90)

    # Format command
    command = f"{knuckle1},{knuckle2},{knuckle3}\n"

    try:
        ser.write(command.encode())  # Send over USB
        print(f"[{datetime.now()}] 📨 Received API Request: {data}")
        print(f"[{datetime.now()}] 🚀 Sending over USB: {command.strip()}")
        return Response({"message": f"Sent positions: {command}"})
    except Exception as e:
        print(f"[{datetime.now()}] ❌ USB Send Error: {e}")
        return Response({"error": str(e)}, status=500)
