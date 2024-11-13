# Litter-Picking and Disposal Robot Project

## Project Overview

This project aims to develop an autonomous three-wheeled litter-picking and disposal robot equipped with a forklift-like actuator arm. Designed to navigate a variety of outdoor terrains, the robot will autonomously locate, pick up, and dispose of litter, contributing to environmental cleanup efforts in urban, park, and natural settings. The robot’s design supports efficient mobility and manipulation capabilities, enabling it to interact with objects of various sizes and shapes.

## Key Features

- **Three-Wheeled Adaptive Locomotion System**  
  The robot uses three wheels for optimal stability and mobility across different surfaces, including paved paths, grass, and light gravel. The wheels are designed to provide energy-efficient and smooth movement on uneven terrain.
  - **Drive Wheels**: Enable efficient, straightforward movement on flat and mildly uneven surfaces.
  - **Turning Mechanism**: The three-wheel configuration allows for tight, precise turns, helping the robot navigate confined spaces and densely littered areas.

- **Forklift-Style Actuator Arm**  
  The robot is equipped with a forklift-like arm designed to pick up litter of varying shapes and sizes. The arm can be lowered to ground level for easy collection and raised to place the litter in an onboard disposal bin or a designated trash collection area.
  - **Gripping Mechanism**: Adaptable gripper for securely handling items such as bottles, cans, and small bags.
  - **Forklift Motion**: The arm moves vertically, enabling the robot to scoop, lift, and release objects with precision and ease.

- **Computer Vision & Autonomous Navigation**  
  The robot utilizes computer vision to detect and identify litter items and assess terrain conditions. It employs AI-driven algorithms to differentiate between litter and natural objects (e.g., rocks, plants) and navigates to each item autonomously, reducing the need for human supervision.
  - **YOLOv8 Nano for Litter Detection**: Using YOLOv8 Nano, a lightweight and efficient object detection model, the robot can accurately identify common litter types while minimizing processing demands. This model enables real-time object detection essential for autonomous litter collection.
  - **Obstacle Avoidance**: Built-in sensors prevent collisions and ensure the robot navigates around obstacles safely.

- **Streaming & Hosting Infrastructure**  
  - **FastAPI for Streaming**: A FastAPI application handles real-time video streaming, allowing the robot’s live feed to be monitored remotely. FastAPI provides a fast, asynchronous framework that integrates well with the YOLOv8 detection pipeline, ensuring efficient processing and streaming performance.
  - **Docker & Proxmox for Hosting**: The robot’s software stack, including YOLOv8 and FastAPI, is containerized with Docker, ensuring consistency and portability. The containers are hosted on Proxmox, providing a stable and scalable environment for managing multiple robots, streaming video, and processing data.

- **Autonomous and Remote Control Modes**  
  The robot can operate autonomously, cleaning designated areas independently, or can be manually controlled through a web-based dashboard for precise operations in complex environments.

## Applications

- **Urban Cleanup**: Collecting litter in city parks, sidewalks, and public areas.
- **Environmental Conservation**: Assisting in maintaining cleanliness in natural reserves, beaches, and trails.
- **Event Spaces**: Keeping large event areas clean, minimizing the need for manual cleanup.
- **Educational Demonstrations**: Showcasing robotics in environmental sustainability and cleanup projects.

## Additional Features (Future Enhancements)

- **Expanded Litter Identification**: Upgrading the computer vision system to recognize a wider range of litter types.
- **Investigation of SLAM (Simultaneous Localization and Mapping)**: We aim to explore SLAM capabilities to improve the robot’s navigation and mapping. Potential options include using LiDAR or integrating OpenCV libraries to provide real-time mapping for autonomous navigation and obstacle handling.

This setup with YOLOv8 Nano, FastAPI, Docker, and Proxmox provides a robust, scalable, and efficient framework for the robot's vision and control systems, making it an effective solution for autonomous litter collection and environmental maintenance.
