# Base image with Python and OpenCV support
FROM python:3.9-slim

# Install system dependencies
RUN apt-get update && \
    apt-get install -y libgl1 libglib2.0-0 openssl && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy SSL certificates
COPY key.pem cert.pem /app/

# Copy the YOLO model file
COPY app/trained_yolov8_model.pt /app/

# Copy application code
COPY app/ /app/

# Copy requirements.txt and install Python dependencies
COPY requirements.txt /app/
RUN pip install -r requirements.txt

# Set environment variable for the YOLO model path
ENV YOLO_MODEL_PATH="/app/trained_yolov8_model.pt"

# Expose the application port
EXPOSE 49088

# Run FastAPI application with SSL using key.pem directly
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "49088", "--ssl-keyfile", "./key.pem", "--ssl-certfile", "./cert.pem", "--ssl-keyfile-password", "123456"]
