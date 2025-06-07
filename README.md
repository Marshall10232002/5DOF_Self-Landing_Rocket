# 🚀 Advanced Controller Design: Gain-Scheduled LQR for a Self-Landing Rocket

This project implements an advanced control strategy for a self-landing rocket using a **Gain-Scheduled Linear Quadratic Regulator (LQR)** combined with a **Disturbance Observer (DOB)**. The controller accounts for time-varying mass and dynamic uncertainties such as wind disturbances and low-bandwidth actuator dynamics.

## 🔧 Project Structure

```
.
├── figures/                # Figures used in the report
├── matlab_code/            # All MATLAB scripts and functions
│   ├── main_simulation.m   # Main entry point for running the rocket simulation
│   ├── controller_design/  # Gain-scheduling, DOB, and LQR tuning logic
│   ├── rocket_dynamics/    # Plant model, actuator dynamics, and wind disturbance
│   └── plots/              # Scripts for plotting results
├── report/                 # IEEE format report files (PDF and LaTeX source)
│   ├── final_report.pdf
│   └── main.tex
└── README.md               # This file
```

## ✨ Features

- ✅ Gain-Scheduled LQR based on real-time rocket mass and thrust
- ✅ Disturbance Observer for estimating external wind forces
- ✅ Thrust Vector Control (TVC) actuator modeled with low-pass filter dynamics
- ✅ Simulation in time-varying wind conditions (typhoon-style)
- ✅ MATLAB-based simulation and post-processing

## 📂 How to Run

### Prerequisites
- MATLAB R2022b or newer
- Control System Toolbox
- Signal Processing Toolbox

### Run Simulation

```matlab
cd matlab_code
main_simulation
```

### Output
- Plots will be generated showing:
  - Tracking performance
  - Disturbance estimation
  - Rocket position and velocity profiles
  - Comparison across disturbance scenarios

## 📊 Results

Simulation results indicate that the gain-scheduled LQR + DOB controller significantly improves trajectory tracking and robustness under time-varying disturbances, particularly in challenging wind conditions.

You can see detailed figures and plots in the `figures/` folder or in the final report.

## 📄 Report

The full project report is written in IEEE format and available [here](./report/final_report.pdf). It includes:

- System modeling
- Control design methodology
- Simulation results
- Discussion on robustness and limitations

## 📌 References

- Zhou, Kemin, et al. *Robust and Optimal Control*. Prentice Hall, 1996.
- Kwon, Wook Hyun, and Soo Hee Han. *Receding Horizon Control: Model Predictive Control for State Models*. Springer, 2005.

## 🔗 License

This project is for academic and non-commercial use only.

## 📬 Contact

Feel free to reach out for collaboration or questions:
**Marshall Chang**  
📧 b10502039@ntu.edu.tw
**Zih-Ying Lee**  
📧 b10502035@ntu.edu.tw
