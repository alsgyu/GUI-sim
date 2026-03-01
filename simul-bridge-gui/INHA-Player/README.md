
</div>

<div align="center">

</br>

**"The Core Intelligence Unit of INHA United"**

The **INHA Player** is the unified executable that drives our autonomous humanoid robots. It serves as a flexible framework capable of dynamically assuming any tactical role on the field—Striker, Defender, or Goalkeeper—based on configuration and game context.

</div>

---

## Architecture

The Player module is built on a **Hyper-Modular Architecture** that strictly separates strategic intent from mechanical execution. This allows for rapid behavior adjustments without core code modification.

1.  **Strategy Layer (The Director)**: Analyzes the global game state (score, time, ball position) to determine the team's overall mode (e.g., *All-Out Attack*, *Deep Defense*).
2.  **Tactics Layer (The Tuner)**: Translates the strategy into concrete parameters (e.g., *sprint speed*, *defense line height*, *shoot threshold*) and injects them into the behavior tree.
3.  **Execution Layer (The Engine)**: Pure functional nodes that execute actions based on the injected parameters, ensuring consistent physical performance regardless of the high-level strategy.

---

## Roles

This single unified codebase supports all field positions:

*   **Striker**: aggressive dribbling, shooting, and space penetration.
*   **Defender**: line control, interception, and clearing.
*   **GoalKeeper**: shot blocking, positioning, and goal defense.

---

## Contribution

This project contributes to the field of humanoid robotics by:

1.  **Robust Autonomy**: Demonstrating behavior trees in dynamic, chaotic environments.
2.  **Unified Framework**: Providing a single, extensible C++ codebase for diverse robot roles.
3.  **Open Source**: Offering a modular research platform for the RoboCup community.

---

<br>

<div align="center">
    <b>Built by INHA United</b><br>
    <i>Pushing the boundaries of Autonomous Soccer</i>
</div>
