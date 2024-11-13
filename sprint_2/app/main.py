from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from ultralytics import YOLO
import cv2
import numpy as np
import base64

app = FastAPI()

# Load the YOLOv8 model
model = YOLO("trained_yolov8_model.pt")

# Serve the HTML page
@app.get("/")
async def get():
    return HTMLResponse(open("templates/index.html").read())

# WebSocket endpoint for streaming video frames
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Receive the base64 image data from the client
            data = await websocket.receive_text()
            image_data = base64.b64decode(data.split(",")[1])
            np_arr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Perform YOLO inference
            results = model(frame)
            annotated_frame = results[0].plot()

            # Encode annotated frame as JPEG
            _, buffer = cv2.imencode('.jpg', annotated_frame)
            encoded_image = base64.b64encode(buffer).decode('utf-8')
            await websocket.send_text(f"data:image/jpeg;base64,{encoded_image}")
    except WebSocketDisconnect:
        print("Client disconnected")

# Function to return the passphrase for the SSL key
def get_ssl_passphrase():
    return "123456"

# Run the FastAPI server with SSL
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=49088,
        ssl_keyfile="./key.pem",
        ssl_certfile="./cert.pem",
        ssl_keyfile_password=get_ssl_passphrase,
    )
