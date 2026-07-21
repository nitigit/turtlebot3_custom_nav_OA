# TurtleBot3 custom Obstacle Avoidance and Mapping System

A ROS2-based autonomous navigation stack implemented on a TurtleBot3 Burger. The system constructs a 2D map using LiDAR and navigates dynamic environments.


# 🚀 Visual Demonstration
https://github.com/user-attachments/assets/814f8ab2-014a-4d8d-9e41-fb2fe3b8d788


# 🛠️ Key Engineering Features
* **LiDAR SLAM:** Built a robust 2D static map using Cartographer.
* **Path Planning:** Implemented Djikstra for local exploration.
* **Obstacle Avoidance:** Uses Artifical Potential Field for basic obstacle avoidance.
* **Hardware:** TurtleBot3 Burger, Raspberry Pi 4, 360 LiDAR.


# 💻 How to Run (Linux/Ubuntu 22.04)
1. Follow the intialization steps from the TurtleBot3 manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Clone the repository: git clone https://github.com/nitigit/turtlebot3_custom_nav_OA.git
3. Build the workspace: colcon build
4. Launch the two controllers:
	ros2 launch apf_controller.py
	ros2 launch dijkistra_explorer.py
	
