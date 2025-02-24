import axios from "axios";

const API_URL = "http://127.0.0.1:8000/api/move/";

export const sendPosition = async (positions: { knuckle1: number; knuckle2: number; knuckle3: number }) => {
    try {
        const response = await axios.post(API_URL, positions);
        console.log("Sent positions:", response.data);
    } catch (error) {
        console.error("Error sending positions:", error);
    }
};
