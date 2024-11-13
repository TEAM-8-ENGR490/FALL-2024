from ultralytics import YOLO
import streamlit as st
import numpy as np
from PIL import Image

# Set the Streamlit page configuration
st.set_page_config(page_title="YOLOv8 Webcam Inference", layout="wide")

# Load the YOLOv8 model (make sure it's for inference only)
model = YOLO('trained_yolov8_model.pt')

# Streamlit title and description
st.title("Team 8 - CNN testing DEMO")


# Use Streamlit's camera input
camera_input = st.camera_input("Capture a frame")

if camera_input:
    # Convert the captured image to a format usable by YOLO
    image = Image.open(camera_input)
    frame = np.array(image)

    # Perform inference using the loaded model
    results = model(frame)

    # Draw bounding boxes and labels on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame using Streamlit
    st.image(annotated_frame, channels="BGR", caption="YOLOv8 Inference Results")