<div align="center">

# :soccer: INHA Soccer

</br>

Hi 👋 We are Inha-United !
Inha-United is a team competing in the RoboCup Soccer Humanoid League.

The **INHA Soccer** provides a comprehensive set of algorithms required for a robot to autonomously perform a soccer game.

Starting from the demo provided by Booster Robotics, we have modularized the system, extended its functionality, and continuously improved its performance through our own research and development efforts.


</div>

---
## System Overview 
<p align="center">
  <img width="3024" height="1274" alt="image" src="https://github.com/user-attachments/assets/74b21e0b-c122-4736-ac74-c55f783e5129" />



</p>

---
## Vision 📷

The vision pipeline of this system is based on a YOLOv8 object detection model and performs high-speed inference through TensorRT optimization.
For the detected ball, line markers, and robot objects, the system applies a camera model and geometric transformations to convert the detection results into positions in the robot coordinate frame.

[More on Vision](https://github.com/Inha-united-soccer/INHA_Vision)

---
## Brain 🧠

The Brain module consists of the following three core functionalities.

### 1. Behavior
Based on a BehaviorTree framework, various behaviors required to perform a soccer game are implemented in a modular manner. These behaviors are composed to form a pipeline that controls the overall game flow.

[More on Striker](https://github.com/Inha-united-soccer/INHA_Striker)

[More on Defender](https://github.com/Inha-united-soccer/INHA_Defender)

[More on GoalKeeper](https://github.com/Inha-united-soccer/INHA_GoalKeeper)


###	2. Localization
Localization algorithms are implemented to estimate the robot’s position and orientation on the field, using information provided by the Vision module.

[More on Localization](https://github.com/Inha-united-soccer/INHA_Localization)


###	3. Communication
The module processes information exchanged between team robots as well as game state data from the Game Controller, and integrates this information with the Behavior module to enable advanced team strategies and decision-making.

---
## game_controller 🎮
The system receives game control data transmitted via the UDP protocol from the Game Controller, converts it into ROS2 topic messages, and publishes them for use by the Brain module.

---
## Interface 💬
The Interface directory defines the ROS message types and communication interfaces used for interactions between modules such as Vision, Brain, and Control.

---

<div align="center">
    <b>Built by INHA United</b><br>
    <i>Pushing the boundaries of Autonomous Soccer</i>
</div>
