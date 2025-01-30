# Autonomous_Project_CARLA
An autonomous driving project using the CARLA simulator


# Project Instructions

## 🚀 Prerequisites
- **Docker** must be installed.
- **CARLA Simulator** must be installed.

## 🏁 Steps to Run the Project

### 1️⃣ Start CARLA Simulator
- Navigate to your **CARLA installation folder**.
- **Double-click** on `Carla.exe` (or run `./CarlaUE4.sh -opengl` on Linux/macOS).
- Wait for **CARLA** to fully load.

### 2️⃣ Clone the Repository (If Not Already Cloned)
Open a terminal and run:
```bash
git clone https://github.com/sarosh2/Autonomous_Project_CARLA.git
cd <your-project-directory>
```

### 3️⃣ Build and Run Docker Services
To set up the necessary Docker containers, follow these steps:

- **Build** the Docker images:
  ```bash
  docker compose build


- Start all the services:
```bash
docker compose up
```

### 4️⃣ Run the AI Test Script
Open a new terminal.
Run the following command to execute the AI test for a specific milestone:
```bash
python MAPE-K_Loop/ai_test.py -m <milestone_number>
```
Replace <milestone_number> with the milestone you want to run.

