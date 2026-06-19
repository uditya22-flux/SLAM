# Horizon Rover Localization System

Welcome to the **horizon_localization** repository. This workspace handles the positional tracking, sensor fusion, and coordinate mapping pipelines for the rover platform. The system is built on ROS 2 to ensure modular, low-latency spatial awareness.

---

## 📂 Repository Directory Structure & Folder Purposes

```text
horizon_localization/
├── docs/                                 # Central team documentation, reports, and architecture diagrams
└── ros2_ws/                              # Primary ROS 2 workspace folder where all custom packages are compiled
    └── src/
        ├── horizon_global_fusion/         # Package for absolute world tracking and global sensor fusion layers
        ├── horizon_localization_bringup/ # Meta-package responsible for orchestration, filtering, and system lifecycle                  
        │
        └── horizon_localization_core/    # Package for local relative tracking, local odometry, and base sensor processing                  # Implements local listener configurations, custom callbacks, and frame math