# 🚀 Advanced Controller Design: Gain-Scheduled LQR for a Self-Landing Rocket

This project implements a control strategy for a self-landing rocket using a **Gain-Scheduled Linear Quadratic Regulator (LQR)** and a **Disturbance Observer (DOB)**. The system is tested under challenging conditions such as mass variation and wind disturbances.

## ✨ Features

- Gain-Scheduled LQR for handling time-varying rocket mass
- Disturbance Observer for external force estimation
- TVC actuator modeled with low-pass dynamics
- MATLAB-based simulation

## 📂 How to Run

1. Run `gain_table_generation.m` to generate the gain schedule table.
2. Run `observer_gain_table_generation.m` to generate the observer gain schedule.
3. Run `main_simulation.m` to simulate the full rocket landing scenario.

All files are located in the `matlab_code` folder.

## 📄 Report

The final project report is available in IEEE format under the `report/` folder.

## 👥 Authors

- **Marshall Chang** — b10502039@ntu.edu.tw
- **Zih-Ying Lee** — b10502035@ntu.edu.tw

## 📬 Contact

For questions or collaboration, please feel free to contact either author via the listed emails.
