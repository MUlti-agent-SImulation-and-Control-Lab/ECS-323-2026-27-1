# Neural-Network-Based Control of a Cable-Driven Soft Robotic Finger on an Arduino

**Context:** Thesis Objective 3 ("Single Finger Prototype & Testing") of *Development of Soft Robotic Grippers: Materials, Control, and Applications* (N. Mehra, Comprehensive Exam, 2 July 2025)

**Status:** Open-loop characterisation done. Neural network (NN) controller designed and ready to implement.

This project characterizes a cable-driven soft robotic finger. A servo pulls the cable to bend the finger, and a flex sensor measures the resulting finger angle. An Arduino logs both signals over time while the servo sweeps back and forth.

The MATLAB script finds each rising, falling, and flat part of the logged data on its own. It fits a straight line between servo angle and finger angle for each ramp. The result is a simple linear model that predicts finger position from the servo command.

**Student Name:** Soham Saha\
**Roll No.:** 24332\
**Course:** ECS323 

## Requirements

- MATLAB R2019b or later
- No extra toolboxes

## Project layout

```
.
├── src/
│   └── Linear_approx_fit.m   # main script
├── logs/
│   ├── pinn_data.csv         # input data: time_ms, servo_angle, finger_angle
│   └── step_response.xlsx    # step-response log
├── results/                  # saved outputs
└── report.pdf
```

## Run

1. Open MATLAB.
2. Set the working folder to `logs/`, because the script loads `pinn_data.csv` from the current folder:

   ```matlab
   cd path/to/project/logs
   ```

3. Run the script:

   ```matlab
   run ../src/Linear_approx_fit.m
   ```

Or run it from the command line:

```bash
cd logs
matlab -batch "run('../src/Linear_approx_fit.m')"
```

## Output

- A figure window shows the finger angle over time, with a fitted line for each segment.
- The Command Window prints a summary for each segment: the linear model (`finger = m·servo + c`, with R²) or the constant level of a flat segment.

## Using your own data

Replace `logs/pinn_data.csv` with a CSV that has these columns:

| Column         | Unit | Description           |
|----------------|------|-----------------------|
| `time_ms`      | ms   | Timestamp             |
| `servo_angle`  | deg  | Servo command angle   |
| `finger_angle` | deg  | Measured finger angle |

To use a different file name, edit `filename` at the top of `src/Linear_approx_fit.m`.

## Parameters

Edit these values in `src/Linear_approx_fit.m`:

| Parameter         | Default | Effect                                     |
|-------------------|---------|--------------------------------------------|
| `smoothWin`       | 101     | Noise smoothing window (samples)           |
| `slopeWin`        | 150     | Half-window for slope estimation (samples) |
| `slopeThresh`     | 3e-4    | Rise/fall vs. flat threshold (deg/ms)      |
| `minSegMs`        | 9000    | Shortest segment kept (ms)                 |
| `forceContinuous` | true    | Joins adjacent fit lines at boundaries     |
