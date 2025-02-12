import React, { useState, useEffect } from "react";
import axios from "axios";
import "./App.css";

const App: React.FC = () => {
    const [knuckle1, setKnuckle1] = useState(90);
    const [knuckle2, setKnuckle2] = useState(90);
    const [knuckle3, setKnuckle3] = useState(90);
    const [mode, setMode] = useState<"manual" | "webcam">("manual"); // Mode toggle
    const [streaming, setStreaming] = useState(false);

    // Update backend with new positions (USB + visualization)
    const updateFingerPosition = () => {
        axios.post("http://127.0.0.1:8000/api/move/", { knuckle1, knuckle2, knuckle3 })
            .then(response => console.log("Command Sent:", response.data))
            .catch(error => console.error("Error:", error));
    };

    // Toggle between manual and webcam modes
    const toggleMode = () => {
        if (mode === "manual") {
            setMode("webcam");
            startHandTracking();
        } else {
            setMode("manual");
            stopHandTracking();
        }
    };

    // Start Webcam Hand Tracking
    const startHandTracking = () => {
        setStreaming(true);
        axios.post("http://127.0.0.1:8000/api/start-stream/")
            .then(response => console.log("Started Webcam Tracking"))
            .catch(error => console.error("Error starting webcam tracking:", error));
    };

    // Stop Webcam Hand Tracking
    const stopHandTracking = () => {
        setStreaming(false);
        axios.post("http://127.0.0.1:8000/api/stop-stream/")
            .then(response => console.log("Stopped Webcam Tracking"))
            .catch(error => console.error("Error stopping webcam tracking:", error));
    };

    // Fetch real-time hand tracking angles from the backend
    useEffect(() => {
        if (mode === "webcam") {
            const interval = setInterval(() => {
                axios.get("http://127.0.0.1:8000/api/finger_angles/")
                    .then(response => {
                        setKnuckle1(response.data.knuckle1);
                        setKnuckle2(response.data.knuckle2);
                        setKnuckle3(response.data.knuckle3);
                    })
                    .catch(error => console.error("Error fetching finger angles:", error));
            }, 100); // Update every 100ms
            return () => clearInterval(interval);
        }
    }, [mode]);

    const svgStyle = { transition: "transform 0.1s linear" };

    return (
        <div className="app-container">
            <h1>Robotic Finger Control</h1>

            {/* Mode Toggle Button */}
            <button className="toggle-mode-button" onClick={toggleMode}>
                {mode === "manual" ? "Switch to Webcam Tracking" : "Switch to Manual Control"}
            </button>

            {/* Webcam Feed (only in webcam mode) */}
            {mode === "webcam" && (
                <div className="webcam-container">
                    <img
                        src="http://127.0.0.1:8000/api/video_feed/"
                        alt="Webcam Feed"
                        style={{
                            width: "400px",
                            borderRadius: "8px",
                            boxShadow: "0px 0px 10px rgba(0, 0, 0, 0.5)",
                            marginBottom: "20px",
                        }}
                    />
                </div>
            )}

            {/* Finger Visualization */}
            <svg width="200" height="400" viewBox="0 0 200 400" style={{ margin: "0 auto", display: "block" }}>
                <rect x="90" y="300" width="20" height="60" fill="gray" />
                <g transform={`translate(100, 300) rotate(${knuckle1 - 90})`} style={svgStyle}>
                    <rect x="-10" y="-50" width="20" height="50" fill="lightgray" />
                    <g transform={`translate(0, -50) rotate(${knuckle2 - 90})`} style={svgStyle}>
                        <rect x="-5" y="-50" width="10" height="50" fill="lightgray" />
                        <g transform={`translate(0, -50) rotate(${knuckle3 - 90})`} style={svgStyle}>
                            <rect x="-3" y="-50" width="6" height="50" fill="lightgray" />
                        </g>
                    </g>
                </g>
            </svg>

            {/* Sliders (only in manual mode) */}
{mode === "manual" && (
    <div className="slider-container">
        <label>Knuckle 1: {knuckle1}°</label>
        <input
            type="range"
            min="0"
            max="180"
            value={knuckle1}
            onChange={(e) => setKnuckle1(Number(e.target.value))}
            onMouseUp={updateFingerPosition} // Trigger command on slider release
        />

        <label>Knuckle 2: {knuckle2}°</label>
        <input
            type="range"
            min="0"
            max="180"
            value={knuckle2}
            onChange={(e) => setKnuckle2(Number(e.target.value))}
            onMouseUp={updateFingerPosition}
        />

        <label>Knuckle 3: {knuckle3}°</label>
        <input
            type="range"
            min="0"
            max="180"
            value={knuckle3}
            onChange={(e) => setKnuckle3(Number(e.target.value))}
            onMouseUp={updateFingerPosition}
        />
    </div>
)}

        </div>
    );
};

export default App;
