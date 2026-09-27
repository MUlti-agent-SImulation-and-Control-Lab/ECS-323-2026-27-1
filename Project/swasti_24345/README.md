**SLIDING MODE CONTROL AND ITS APPLICATIONS**

Course: ECS323 Control Systems
Mid-semester Project Evaluation

Team Members:
Ishaan Jena(24409)
Swasti Rohliyan(24345)

This project attempts to understand the function and applications of Sliding Mode Control. 
Sliding Mode Control (SMC) nonlinear method of robust control that forces a system to "slide" along a plane, called the sliding plane.
The implementation of SMCs is carried out through the use of some standard parameters such as:
x_d : the desired trajectory
e : error between the reference and the input signal
s : the sliding plane, which is expressed as a function between two errors.

Contents of the Repository:
README.md : overview of the project
pendulum_smc.py : A python implementation of an SMC plant, taking a Simple Pendulum under gravity as the model
smc1.slx : a MATLAB SimuLink implementation of an SMC system using a sine wave
Square.m : A Level-2 S function in MATLAB, that generates a Square wave output witha Sine Wave input, that can me used to generate a square wave disturbance in an SMC model.

Requirements for the code:
numpy and matplotlib libraries to be installed in the system, which can be done through the command
pip install numpy matplotlib
in the command prompt
MATLAB to be installed alongwith the simulink package, to run the simulation

Limitations: 
The method of SMC introduces chatter, which is a high frequency oscillation around the sliding surface that arises due to discontinuous control and imperfections in its digital implementation.
The Python model attempts to reduce this chattering by using the tanh() method, which is a continuous approximation, hence only reduces the chattering while preventing it from reaching total zero.
The SimuLink model attempts to mitigate this problem by using the signum() function, which has a more rigid boundary layer.
The Chattering problem can be understood as a trade-off between chattering and tracking precision.
A high error boundary layer would eliminate chatter but degrade the tracking precision, resulting in deviation of the path significantly from the sliding path.
A narrower boundary layer would result in an increase in high-frequency chattering while improving significantly the tracking precision.

Future Plans:
Moving ahead, this project plans to explore the various methods of chattering reduction and methods of implementation in the models.
Moreover, explore novel applications of SMC and models that improve upon it, in the fields of laser communication and satellite telemetry.

Reference Papers:
Control Technology of Ground-Based Laser Communication Servo Turntable via a Novel Digital Sliding Mode Controller, Zhang et al.
 https://www.mdpi.com/2076-3417/9/19/4051 
A Composite Control Method Based on Model Predictive Control and a Disturbance Observer for the Acquisition, Tracking, and Pointing System, Xie et al.
 https://www.mdpi.com/2076-0825/13/10/417 
 
