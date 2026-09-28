
# Vision-Guided Autonomous Mobile Dustbin

## ECS-323 Course Project — Midsem - Sahil Pandey 23219 & Rahul Mishra 23258

### Current milestone
This version focuses only on the first subsystem of the proposed autonomous dustbin:

1. Simulate the path of waste that has been thrown.
2. Produce noisy measurements to simulate imperfect sensing.
3. Make an estimate of the trajectory based on the measurements.
4. Estimate where the trash will land.

The following items are not part of this milestone: robot position control, three-wheel motion control, feedback control, and automatic lid control. They will be included in the subsequent milestones.

## Requirements

- MATLAB R2022b or later
- This milestone does not require any external MATLAB toolbox.

## Repository structure

```text
sahil_23219/
├── README.md
├── requirements.txt
├── src/
│   ├── main.m
│   ├── simulate_trajectory.m
│   └── predict_landing_point.m
├── logs/
├── results/
    └── trajectory_prediction.png   # this is the output image after the matlab main.m code 
└── report/
    └── report.pdf
```

## How to run

1. Open MATLAB.
2. Select the src folder to make it the current MATLAB folder.
3. Run:

```matlab
main
```

4. The program will:
   - simulate the trash trajectory,
   - add measurement noise,
   - estimate the trajectory,
   - calculate the predicted landing point,
   - display the result,
   - save the figure with the name `results/trajectory_prediction.png`.

When the project is opened from the root of the repository, the script will automatically save the results in the `results` folder of the repository.

## Mathematical model

The simulated horizontal motion is:

x(t) = x0 + vx0*t

The vertical motion is:

z(t) = z0 + vz0*t - (1/2)*g*t^2

where:

- x0 = initial horizontal position
- z0 = initial height
- vx0 = initial horizontal velocity
- vz0 = initial vertical velocity
- g = gravitational acceleration

The landing time is obtained from:

z(t_land) = 0

The predicted landing position is then:

x_land = x0 + vx0*t_land

A quadratic model is fitted to the measured vertical trajectory in the case of the noisy measurements, and the positive root which relates to the future point of intersection with the ground is used to estimate the landing time.

## Expected output

The main result is a plot containing:

- true trash trajectory,
- noisy sensor measurements,
- estimated trajectory,
- predicted landing point.

## Future milestones

- Three-wheel mobile dustbin kinematic model
- Position-control system
- PID/LQR controller
- Closed-loop simulation
- Automatic lid control
- Disturbance and performance analysis
