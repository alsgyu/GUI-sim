<div align="center">
  
# :soccer: INHA GoalKeeper
**Advanced Autonomous Agent for Humanoid Soccer**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-3490dc.svg?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![BehaviorTree](https://img.shields.io/badge/BehaviorTree-V4-2ca02c.svg?style=for-the-badge)](https://www.behaviortree.dev/)
[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg?style=for-the-badge)](LICENSE)

*Cognitive Flexibility • Dynamic Positioning • Consistency and Safety*

</br>

**"To create a soccer-playing intelligence that doesn't just calculate, but *understands* the flow of the game."**

The **INHA GoalKeeper** is designed to bridge the gap between rigid robotic control and dynamic human intuition. By leveraging hierarchical behavior trees and advanced motion planning, our agent demonstrates adaptive gameplay—ball-trajectory prediction–based positioning, proactive blocking and saving in 1v1 situations, and safe clearing in high-risk areas.
</div>

---

## Key Features

### **Cognitive Flexibility**
Instead of simple if-else logic, we utilize a **Behavior Tree (BT)** architecture that allows for complex, reactive decision-making. The robot constantly evaluates the game state to transition between behaviors seamlessly.
*   **Reactive**: Handles interruptions (e.g., sudden ball loss) gracefully.
*   **Modular**: Easy to expand with new strategies or plays.

### **Dynamic Positioning**
Rather than holding a fixed spot in front of the goal, the goalkeeper **optimizes its coverage angle and distance.**
*   **Prediction-Based Adjustment**: Proactively adjusts its line and angle based on the ball’s speed and direction
*   **Shot-Angle Coverage**: Dynamically maintains goal-covering positions to reduce the shooter’s available angles

### **Consistency & Safety**
Under pressure, the goalkeeper **prioritizes reliably neutralizing threats** and minimizing risk.
*   **Decisive Clearing**: If a dangerous situation is detected, the goalkeeper clears the ball to a safe area.
*   **Safe Clearing Approach**: For clearance, the goalkeeper approaches the ball safely using curved paths when needed.
  
---

## GoalKeeper Behavior Tree Overview

The goalkeeper’s decision-making framework is composed of three high-level states: **Hold**, **Clearing**, and **Find**.

* **Hold**:
In this state, the robot predicts the ball trajectory and continuously computes and moves to the optimal position that minimizes the opponent’s shooting angle.

* **Clearing**:
When the ball enters a critical area, the goalkeeper performs a clearing action, kicking the ball away toward the direction opposite the goal.

* **Find**:
If the ball position is lost, the robot combines head rotation and body rotation to obtain an omnidirectional field of view and re-locate the ball efficiently.

The detailed system architecture is illustrated in the figure below.
<p align="center">
  <img width="700" alt="goalkeeper" src="https://github.com/user-attachments/assets/bfe44bd2-ee73-400d-8f71-bbb564fad0b3" />
</p>

---

<div align="center">
    <b>Built with by INHA United</b><br>
    <i>Pushing the boundaries of Autonomous Soccer</i>
</div>
