Robotic Finger Control System

📌 Overview

This project enables real-time control of a robotic finger with three knuckles using a React frontend and a Django backend. The system sends USB commands to move servos that control each knuckle. It also supports webcam-based tracking of the user's index finger using OpenCV and Mediapipe.

📂 Project Structure

Folder / File

Description

backend/

Django backend to process commands and communicate over USB.

frontend/

React-based UI to control the robotic finger with sliders or webcam tracking.

backend/robot/views.py

API endpoint to receive commands, process webcam data, and send them to the robotic finger.

frontend/src/App.tsx

Main React UI with sliders and live webcam tracking.

frontend/src/Api.ts

Handles API communication between React and Django.

requirements.txt

Lists required Python dependencies for both backend and webcam tracking.

README.md

Project documentation and quickstart guide.

⚙️ How It Works

1️⃣ User Interaction (Frontend)

The user chooses between Manual Control (sliders) or Webcam Tracking.

In Manual Mode, the React frontend sends a POST request with the knuckle positions to the Django backend.

In Webcam Mode, Mediapipe processes the user's hand in real-time to calculate the finger joint angles, which are sent to the robotic finger.

2️⃣ Backend Processing (Django)

The Django API receives the knuckle angles and formats them as a string (e.g., "90,120,150\n").

It sends the command via USB to the robotic finger and prints debug logs.

Webcam frames are processed with OpenCV and Mediapipe to extract real joint angles for the index finger.

3️⃣ Robotic Finger Execution

An Arduino (or microcontroller) reads the USB command.

It parses the angles and moves the servos to match.

It sends a confirmation message (e.g., "MOVED 90,120,150") back via USB.

🚀 Quickstart Guide

🔹 Prerequisites

Python 3.9+

Node.js 16+

A connected robotic finger with USB support

🔹 Setup Instructions

1. Install Backend (Django)

cd backend
python -m venv venv
source venv/bin/activate # On Windows: venv\Scripts\activate
pip install -r requirements.txt
python manage.py migrate
python manage.py runserver

The backend will run at http://127.0.0.1:8000.

2. Install Frontend (React)

cd frontend
npm install
npm start

The frontend will run at http://localhost:3000.

📡 API Endpoints

Method

Endpoint

Description

POST

/api/move/

Send knuckle positions to the robotic finger.

GET

/api/video_feed/

Stream live webcam feed.

GET

/api/finger_angles/

Fetch calculated finger joint angles.

Example API Request (Manual Mode)

{
"knuckle1": 90,
"knuckle2": 120,
"knuckle3": 150
}

🔌 USB Commands Sent to Robotic Finger

Knuckle 1

Knuckle 2

Knuckle 3

USB Command

90°

120°

150°

90,120,150\n

110°

140°

170°

110,140,170\n

80°

100°

120°

80,100,120\n

The Arduino (or microcontroller) reads these values and adjusts servos accordingly.

🖥️ Viewing Debug Output

🔹 Django Backend Logs

cd backend
python manage.py runserver

Example Logs:

[2025-02-08 14:35:30] 📨 Received API Request: {"knuckle1": 120, "knuckle2": 135, "knuckle3": 90}
[2025-02-08 14:35:30] 🚀 Sending over USB: 120,135,90
[2025-02-08 14:35:30] 🔄 USB Response: MOVED 120,135,90

🔹 Direct USB Debugging (Python)

To manually read the robotic finger's response, run:

python -m serial.tools.miniterm COM3 9600

Expected Output:

120,135,90
MOVED 120,135,90

🌟 Additional Features

Manual and Webcam Modes:

Switch between sliders and real-time hand tracking.

Hand tracking uses Mediapipe to detect the index finger and calculate joint angles.

Live Visualization:

The robotic finger's movement is displayed as an SVG animation in the frontend.

Cross-Platform Support:

Works on Windows, macOS, and Linux.

✅ System Requirements

Component

Version

Python

3.9+

Node.js

16+

Django

4.2.3

TensorFlow

2.13.0

Mediapipe

0.10.21

OpenCV

4.11.0
