import React, { useState } from "react";
import { sendPosition } from "./Api";
import "./App.css";

const App: React.FC = () => {
    const [knuckle1, setKnuckle1] = useState(90);
    const [knuckle2, setKnuckle2] = useState(90);
    const [knuckle3, setKnuckle3] = useState(90);

    const updateFingerPosition = () => {
        sendPosition({ knuckle1, knuckle2, knuckle3 });
    };

    const resetPosition = () => {
        setKnuckle1(90);
        setKnuckle2(90);
        setKnuckle3(90);
        sendPosition({ knuckle1: 90, knuckle2: 90, knuckle3: 90 });
    };

    return (
        <div className="app-container">
            <h1>Robotic Finger Control</h1>

            {/* Visual representation of the robotic finger */}
            <svg width="200" height="400" viewBox="0 0 200 400">
    {/* Palm */}
    <rect x="90" y="300" width="20" height="60" fill="gray" />

    {/* Knuckle 1 */}
    <g transform={`translate(100, 300) rotate(${knuckle1 - 90})`}>
        <rect x="-10" y="-50" width="20" height="50" fill="lightgray" />
        {/* Knuckle 2 */}
        <g transform={`translate(0, -50) rotate(${knuckle2 - 90})`}>
            <rect x="-5" y="-50" width="10" height="50" fill="lightgray" />
            {/* Knuckle 3 */}
            <g transform={`translate(0, -50) rotate(${knuckle3 - 90})`}>
                <rect x="-3" y="-50" width="6" height="50" fill="lightgray" />
            </g>
        </g>
    </g>
</svg>




            <div className="slider-container">
                <label>Knuckle 1: {knuckle1}°</label>
                <input
                    type="range"
                    min="0"
                    max="180"
                    value={knuckle1}
                    onChange={(e) => setKnuckle1(Number(e.target.value))}
                    onMouseUp={updateFingerPosition}
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

            <button className="reset-button" onClick={resetPosition}>Reset Position</button>
        </div>
    );
};

export default App;
