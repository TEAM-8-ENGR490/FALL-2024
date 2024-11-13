# Use the official Python image as the base image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Copy the requirements and app files
COPY requirements.txt requirements.txt
COPY yolo_streamlit_app.py yolo_streamlit_app.py
COPY trained_yolov8_model.pt trained_yolov8_model.pt
COPY cert.pem cert.pem
COPY key.pem key.pem

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Decrypt the key using openssl and passphrase '123456'
RUN openssl rsa -in key.pem -out key_nopass.pem -passin pass:123456

# Expose the port Streamlit will run on
EXPOSE 8501

# Command to run the app
CMD ["streamlit", "run", "yolo_streamlit_app.py", "--server.address=0.0.0.0", "--server.port=49078", "--server.sslCertFile=cert.pem", "--server.sslKeyFile=key_nopass.pem"]
