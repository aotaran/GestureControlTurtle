 # 🖐️ Gesture-Controlled Turtlesim with ROS2 Humble

This ROS2 package allows you to control two turtles in `turtlesim` using **hand gestures** via a webcam. The **right hand** controls `/turtle1`, and the **left hand** controls `/turtle2`.

## 📌 Features
- Uses **MediaPipe Hands** for real-time gesture recognition.
- Controls two turtles in `turtlesim` based on hand gestures.
- A **single launch file** starts everything (Turtlesim, Spawning, Gesture Control).

---

## 🛠️ Installation & Setup
### 1️⃣ Install ROS2 Humble & Dependencies
Ensure you have **ROS2 Humble** installed. Then, install `turtlesim`:
```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

### 2️⃣ Install Required Python Packages
```bash
pip install mediapipe opencv-python numpy
```

### 3️⃣ Clone & Build the Package
```bash
cd ~/ros2_ws/src
git clone <your-repo-url> gesture_control
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 Running the Project
Start everything (Turtlesim + Gesture Control) with:
```bash
ros2 launch gesture_control gesture_control_launch.py
```
This will:
1. Open **Turtlesim** with one turtle.
2. Spawn a second turtle (`turtle2`).
3. Start the gesture control node.

---

## 🖐️ Hand Gestures
| Gesture | Action (Right Hand → Turtle1, Left Hand → Turtle2) |
|---------|--------------------------------------------------|
| Open hand | Stop |
| Fist | Position Mode |
| Index finger open | Velocity Mode |
| Peace Sign | Rotation Mode |

**Position Mode:** Hand position is in xy-plane translated to Turtle position in xy-plane.

**Velocity Mode:** Hand position is in xy-plane translated to Turtle velocity in xy-plane.

**Rotation Mode:** Hand rotation is translated to Turtle rotation speed. 


**Press 'Q' to quit the gesture recognition window.**


## 📜 License
This project is open-source under the **Apache 2.0**.

---

## 🤝 Contributing
Pull requests are welcome! Feel free to improve gesture detection or add more features. 🚀

---

## 📧 Contact
For questions, reach out via GitHub issues. Happy coding! 🎯

