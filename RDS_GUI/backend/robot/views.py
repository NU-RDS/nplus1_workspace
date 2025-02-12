import cv2
import mediapipe as mp
import serial
import math
from django.http import StreamingHttpResponse, JsonResponse
from rest_framework.decorators import api_view

# Global Variables
SERIAL_PORT = "COM4"
BAUD_RATE = 9600

ser = None
cap = None
hands = None
latest_angles = {"knuckle1": 90, "knuckle2": 90, "knuckle3": 90}
streaming = False


# Initialize Serial Port
def initialize_serial():
    global ser
    try:
        if not ser or not ser.is_open:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print("Serial port opened successfully!")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        ser = None


# Initialize Mediapipe Hand Tracking
def initialize_mediapipe():
    global hands
    if hands is None:
        print("Initializing Mediapipe...")
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(
            static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7
        )
        print("Mediapipe initialized successfully!")


# Proper Angle Calculation Using atan2
def calculate_angle(p1, p2):
    """
    Calculates the angle between two points relative to the horizontal axis.
    Ensures a valid range of [0, 180] degrees.
    """
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    angle = math.degrees(math.atan2(dy, dx))  # atan2 gives the angle in radians

    # Convert angle to [0, 180] range (servo-compatible)
    angle = abs(angle)
    if angle > 180:
        angle = 180  # Ensure the angle never exceeds 180°

    return int(angle)


# Process video frames from webcam
def video_stream():
    global cap, hands, latest_angles, streaming
    initialize_mediapipe()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam!")
        streaming = False
        return

    while streaming:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(
                    frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS
                )

                # Extract joint angles (index finger landmarks)
                index_finger = [
                    hand_landmarks.landmark[5],  # MCP joint
                    hand_landmarks.landmark[6],  # PIP joint
                    hand_landmarks.landmark[7],  # DIP joint
                    hand_landmarks.landmark[8],  # Tip
                ]

                # Compute angles with the fixed function
                latest_angles["knuckle1"] = calculate_angle(
                    index_finger[0], index_finger[1]
                )
                latest_angles["knuckle2"] = calculate_angle(
                    index_finger[1], index_finger[2]
                )
                latest_angles["knuckle3"] = calculate_angle(
                    index_finger[2], index_finger[3]
                )

                # Send angles over USB
                if ser and ser.is_open:
                    command = f"{latest_angles['knuckle1']},{latest_angles['knuckle2']},{latest_angles['knuckle3']}\n"
                    ser.write(command.encode())
                    print(f"✅ Sent over USB: {command}")  # ✅ Debug output

        _, jpeg = cv2.imencode(".jpg", frame)
        yield (
            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
        )

    cap.release()


# Start Webcam Video Feed
@api_view(["GET"])
def video_feed(request):
    global streaming
    streaming = True
    return StreamingHttpResponse(
        video_stream(), content_type="multipart/x-mixed-replace; boundary=frame"
    )


# Stop Webcam Video Feed
@api_view(["POST"])
def stop_video_feed(request):
    global streaming
    streaming = False
    return JsonResponse({"status": "stopped"})


# Get Finger Angles for GUI Update
@api_view(["GET"])
def get_finger_angles(request):
    return JsonResponse(latest_angles)


# Move Finger Manually via Sliders
@api_view(["POST"])
def move_finger(request):
    global ser
    knuckle1 = request.data.get("knuckle1", 90)
    knuckle2 = request.data.get("knuckle2", 90)
    knuckle3 = request.data.get("knuckle3", 90)

    try:
        command = f"{knuckle1},{knuckle2},{knuckle3}\n"
        if ser and ser.is_open:
            ser.write(command.encode())
            print(f"✅ Sent manual command: {command}")
        return JsonResponse({"status": "success", "command": command})
    except Exception as e:
        print(f"Error sending manual command: {e}")
        return JsonResponse({"status": "error", "error": str(e)})


# Initialize Serial at Startup
initialize_serial()
