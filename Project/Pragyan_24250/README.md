# Shared Control of a BCI-Driven Wheelchair

**Pragyan Mohanty** · Roll No. 24250 · Control Systems Project

This repository covers **Stage 1**. The code in `src/` will be uploaded soon.

---

## Problem

Some people with severe paralysis cannot move, but they can still *imagine* moving a hand or a foot. Imagined movement changes the brain's rhythms in a way that scalp EEG can pick up. A brain–computer interface (BCI) can classify those changes and use them to drive a powered wheelchair.

The difficulty is that the signal is unreliable:

- the decoder is often wrong,
- it takes time to reach a decision, and
- it only produces a new command every couple of seconds.

A wheelchair driven directly by such a signal is not safe.

## Objective

Find out how control should be **shared** between the user and an autonomous navigator, so that the chair stays safe and stable while the user keeps as much control as possible.

## Proposed solution

The chair gets its own autonomous controller. The command sent to the motors is a blend of the user's command and the machine's command:

```
u = α · u_BCI + (1 − α) · u_auto        0 ≤ α ≤ 1
```

With α = 1 the user drives alone, and with α = 0 the machine ignores the user. Neither extreme is acceptable. The project studies how α should be chosen. The main candidate makes α depend on how confident the decoder is and on how risky the current situation is (time to collision). It will be compared with fixed blending, and the stability of the delayed loop will be analysed as α changes.

## Data

BCI Competition IV, dataset 2a (also distributed as BNCI2014_001):

- 9 subjects, 22 EEG channels, 250 Hz
- 4 imagined movements: left hand, right hand, feet, tongue
- 2 sessions per subject recorded on different days, 288 trials per session

Each class is mapped to a wheelchair command: left hand → turn left, right hand → turn right, feet → forward, tongue → stop.

- **Dataset:** https://www.bbci.de/competition/iv/#dataset2a
- **Stage 1 report (PDF):** _link to be added_

## Approach

**Stage 1: baseline from real data.** We take the dataset as the foundation. We build a standard decoder on it (8–30 Hz filter, CSP, shrinkage LDA, Platt calibration), train it on day 1 and test it on day 2. From the test session we measure what the controller will depend on later:

| quantity | Stage 1 result (mean of 9 subjects) |
|---|---|
| accuracy | 0.443 (chance 0.25) |
| Cohen's κ | 0.258 |
| calibration error (ECE), before → after | 0.196 → 0.096 |
| confidence separating right from wrong decisions (ROC AUC) | 0.676 |
| decision latency | median 2.0 s to 4.0 s depending on subject; many trials never become confident within 4 s |

The decoder is then **frozen**, and these numbers become the fixed baseline.

**Stage 2: optimisation.** The Stage 1 data is used to build a realistic model of the BCI channel: its error rate, how errors cluster, their direction and their delay. On top of that model, Stage 2 designs and tunes the sharing law. This covers the autonomous controller (LQR / Dynamic Window Approach), the adaptive α, the stability boundary τ_max(α) and a comparison of the different policies in simulation.

Full details and results are in `report.pdf`.

## Repository structure

```
.
├── README.md
├── report.pdf              Stage 1 report
├── report.tex              LaTeX source of the report
├── requirements.txt
├── config/
│   └── default.yaml        all parameters
├── src/
│   ├── decoder/            Stage 1: EEG loading, CSP + LDA, calibration, latency
│   ├── models/             Stage 2: BCI error model, operator model
│   ├── plants/             wheelchair and double-integrator models
│   ├── control/            LQR, DWA, risk signal, arbitration laws
│   ├── sim/                simulation loop and experiments
│   └── analysis/           stability analysis and figures
├── data/
│   └── interchange/        Stage 1 outputs used by Stage 2
├── results/
└── tests/
```

The code will be uploaded soon. Stage 1 will run with:

```bash
pip install -r requirements.txt
python -m src.decoder.run_stage1
```
