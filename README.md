# 🤖 Robot Cleaner - Autonomous Trash Collector

An autonomous Arduino-based robot designed to patrol an area, detect objects ahead, and collect them using an articulated shovel.

## ⚙️ How It Works
* **Patrol (Wander & Scan):** The robot drives straight for 5 seconds and spins in place for 5 seconds while scanning for obstacles.
* **Detection:** When an object is spotted within **30 cm**, the robot verifies the distance and switches to approach mode.
* **Collection:** Once within **15 cm** of the target, the robot stops, smoothly lowers the shovel, pushes forward to scoop the item, lifts the shovel back up, and reverses to resume patrolling.
* **Power Management:** Features staggered motor startups and gradual servo movements to prevent current spikes and conserve battery power.

## 🛠️ Hardware Components
* Arduino Board (Uno / Nano)
* HC-SR04 Ultrasonic Sensor
* 1x Micro Servo Motor (for the shovel)
* 2x DC Gear Motors + H-Bridge Motor Driver (L298N or similar)
* Robot chassis and battery pack

## 🔌 Pinout (Wiring)
| Component | Arduino Pin |
| :--- | :--- |
| **HC-SR04 Sensor (Trig)** | D7 |
| **HC-SR04 Sensor (Echo)** | D8 |
| **Motor A (IN1 / IN2)** | D4 / D5 |
| **Motor B (IN3 / IN4)** | D9 / D10 |
| **Shovel Servo (Signal)** | D6 |
