# BCI-Autonomous Wheelchair Shared Control System

**Pragyan Mohanty** · Roll No. 24250 · Control Systems Project

This repository covers **decoder part**, the control part will be done in post midsem.

---

## Project objective
This project is more about how a brain-computer interface (BCI) can be used as a command channel for a wheelchair.

A BCI can decode a person's intended movement from EEG signals. However, EEG-based commands are not perfectly reliable: the decoder can make error , its confidence may not always represent its actual accuracy, and its performance can change between recording sessions.

The overall project will eventually use this information to design a controller that can share control between the user's BCI command and an autonomous wheelchair controller.



### reference - 
 [1] Tonin, L., & Millán, J. del R. (2021). *Noninvasive Brain–Machine Interfaces for Robotic Devices*. 
Annual Review of Control, Robotics, and Autonomous Systems, 4, 191–214. 
https://doi.org/10.1146/annurev-control-012720-093904 



## Control architecture and  formulation 

The chair gets its own autonomous controller. The command sent to the motors is a blend of the user's command and the machine's command:

```
u = α · u_BCI + (1 − α) · u_auto        0 ≤ α ≤ 1
```

The wheelchair is driven by a blend of two commands: the user's decoded command and the command of an autonomous navigation controller. A single authority parameter α, sets the share each one gets. rather than improvising decoder efficiency (i.e improving alpha),   it will be adapted online from two signals. The first is the decoder's calibrated confidence, which estimates whether the command is what the user intended. The second is a situational risk measure based on time-to-collision, which estimates how costly an error would be at that moment. The planned framework uses LQR and the Dynamic Window Approach for the autonomous layer. It adds hysteresis and rate limiting so α changes smoothly, and a frequency-domain analysis of how much loop delay the system can tolerate as a function of α. This part of the work is in progress.

## Data

BCI Competition IV, dataset 2a (also distributed as BNCI2014_001):

- 9 subjects, 22 EEG channels, 250 Hz
- 4 imagined movements: left hand, right hand, feet, tongue
- 2 sessions per subject recorded on different days, 288 trials per session

Each class is mapped to a wheelchair command: left hand → turn left, right hand → turn right, feet → forward, tongue → stop.

- **Dataset:** https://www.bbci.de/competition/iv/#dataset2a
- **Dataset Collection and Experimental Protocol** - https://www.bbci.de/competition/iv/desc_2a.pdf

## Progress and implemenation

 currently the repository contains only the decoder pipeline where we analyse the accuracy, confidence , Calibration quality ECE, Confusion matrix, Cohen's κ (kappa) , Cohen's κ (kappa) from the bci datasets. 
 
 the next phase we use these saved outputs of the decoder to build the model for bci channel for capturing how often it is wrong, in which direction, and with what delay. On top of that model, the authority share equation  will be designed, including the autonomous controller, an authority parameter α that adapts to decoder confidence and situational risk, and a stability analysis of the delayed feedback loop, all evaluated in simulation.
 

## Repository structure


```
├── stage_1.ipynb           the whole pipeline + figures
├── summary_by_subject.csv  per-subject results
├── figures/                all figures (made by the notebook)
├── requirements.txt
└── .gitignore
```
the structure  is tentative and will change as control and simulation module will be added.


The code will be uploaded soon. Stage 1 will run with:

```bash
pip install -r requirements.txt
python -m src.decoder.run_stage1
```


## Remaining Work

1. **BCI channel model.** Fit a per-subject Markov error model and latency distribution from the Stage 1 outputs.
2. **Plant and operator models.** Implement the wheelchair and double-integrator plants and the delayed-PD operator.
3. **Autonomous controllers.** Implement LQR and the Dynamic Window Approach with obstacle avoidance.
4. **Arbitration policies.** Implement fixed blending, confidence-only, and risk-gated adaptive allocation, $\alpha = c(1 - r)$.
5. **Stability analysis.** Derive $\tau_{\max}(\alpha)$ and $\alpha^*$, and verify them numerically.
6. **Simulation study.** Compare all policies across 9 subjects, 3 environments, and 20 random seeds on success rate, collision rate, and retained user authority.
7. **Final report.** Consolidate results, ablations, and limitations.

## Stage 1 Results

Cross-session evaluation on **BNCI2014_001** (Train: Day 1, Test: Day 2, 4 classes, chance = 0.25)[cite: 3]:

| Subject | Raw Acc | Calib. Acc | $\kappa$ | ECE Raw | ECE Calib. | Median Latency (s) | Censored |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **S1** | 0.576 | 0.556 | 0.407 | 0.138 | 0.150 | 2.00 | 0.12 |
| **S2** | 0.611 | 0.556 | 0.407 | 0.150 | 0.075 | 3.55 | 0.46 |
| **S3** | 0.708 | 0.691 | 0.588 | 0.146 | 0.044 | 2.00 | 0.02 |
| **S4** | 0.389 | 0.389 | 0.185 | 0.142 | 0.077 | 4.00 | 1.00 |
| **S5** | 0.312 | 0.191 | -0.079 | 0.148 | 0.144 | 4.00 | 0.92 |
| **S6** | 0.368 | 0.378 | 0.171 | 0.188 | 0.030 | 4.00 | 0.83 |
| **S7** | 0.469 | 0.396 | 0.194 | 0.299 | 0.188 | 2.00 | 0.02 |
| **S8** | 0.535 | 0.497 | 0.329 | 0.091 | 0.094 | 3.70 | 0.40 |
| **S9** | 0.378 | 0.337 | 0.116 | 0.157 | 0.063 | 4.00 | 0.99 |
| **Mean** | **0.483** | **0.443** | **0.258** | **0.162** | **0.096** | **—** | **0.53** |

