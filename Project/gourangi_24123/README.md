# Distributed Optimization Using Relative Binary State Measurements Only

## Problem Statement

Consider a network of $N$ agents interacting over an **undirected, connected** communication graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$. Each agent $i \in \{1, \dots, N\}$ has a local cost function $f_i : \mathbb{R}^n \to \mathbb{R}$ acting on its own state $x^i \in \mathbb{R}^n$. Writing the aggregate state as $x = (x^1, \dots, x^N)$, the global objective is

$$
\min_{x \in \mathbb{R}^n} f(x) = \sum_{i=1}^{N} f_i(x),
$$

which each agent must solve using **only local information** and **binary** ($\{+1, -1\}$) signals about its relative disagreement with its neighbors — i.e., no continuous-valued state exchange between agents.

### Meaning of consensus

Consensus is classically encoded via the graph Laplacian $L = DD^\top$ (where $D$ is the incidence matrix of $\mathcal{G}$), through the condition $(L \otimes I_n)x = 0$. Equivalently, since $L \otimes I_n = (D \otimes I_n)(D^\top \otimes I_n)$ and $L \otimes I_n \succeq 0$:

$$
(L \otimes I_n)x = 0 \iff \|(D^\top \otimes I_n)x\|^2 = 0 \iff (D^\top \otimes I_n)x = 0.
$$

The **edge-based (incidence) formulation** $(D^\top \otimes I_n)x = 0$ is preferred over the Laplacian formulation because it directly exposes the pairwise disagreements $x^i - x^j$ across each edge $(i,j) \in \mathcal{E}$ — exactly the quantity the nonlinear/binary coupling acts on. Concretely:

$$
(D^\top \otimes I_n)x = 0 \iff x^i = x^j \; \forall (i,j) \in \mathcal{E} \iff x = \mathbf{1}_N \otimes \tau \text{ for some } \tau \in \mathbb{R}^n,
$$

with the last equivalence following from connectivity of $\mathcal{G}$.

### Distributed Optimization 

Under the assumption that the global cost $f$ is strongly convex (with each local cost being strongly convex), the original problem is reformulated as follows:

$$
\min_{x^i \in \mathbb{R}^n} \sum_{i=1}^{N} f_i(x^i) \quad \text{subject to} \quad (D^\top \otimes I_n)x = 0.
$$

This is the basis for the algorithm below. We choose $g(\cdot) = \operatorname{sign}(\cdot)$ to show that the agents can only have binary communciation.

## Algorithm

For each agent $i \in \{1, \dots, N\}$, the proposed dynamics are:

$$
\dot{v}^i = \alpha\beta \sum_{j=1}^{N} a_{ij}\,\operatorname{sign}(x^i - x^j),
$$

$$
\dot{x}^i = -\alpha \nabla f_i(x^i) - \beta \sum_{j=1}^{N} a_{ij}\,\operatorname{sign}(x^i - x^j) - v^i,
$$

where:
- $v^i \in \mathbb{R}^n$ is an internal state of agent $i$,
- $\alpha, \beta \in \mathbb{R}_{>0}$ are static control gains,
- $a_{ij}$ are entries of the adjacency matrix of $\mathcal{G}$,
- each agent only needs the **sign** of its relative disagreement with each neighbor — no magnitude information is exchanged.

In compact network form, with $\mathcal{D} := D \otimes I_n$:

$$
\dot{v} = \alpha\beta\,\mathcal{D}\,\operatorname{sign}(\mathcal{D}^\top x),
$$

$$
\dot{x} = -\alpha \nabla \tilde{f}(x) - \beta\,\mathcal{D}\,\operatorname{sign}(\mathcal{D}^\top x) - v.
$$

## Codebase Structure

| File | Role |
|---|---|
| **`driver.m`** | Top-level entry point. Calls `edge_do_main(alpha, beta, filename)` with specific gain values to reproduce the paper's simulation figures. |
| **`edge_do_main.m`** | Sets up one full experiment: defines the number of agents `n`, state dimension `d`, the communication graph adjacency matrix `A`, the local cost/gradient function handles, and the initial conditions. Builds the incidence matrix via `incidence_matrix.m`, runs the simulation via `forward_euler_edge_do.m`, computes the total cost and summed gradient trajectories, plots agent states / cost / gradient sum, and exports the figure to a vector PDF via `exportgraphics`. |
| **`incidence_matrix.m`** | Builds the $n \times m$ incidence matrix `D` of the undirected graph from its adjacency matrix `A`, orienting every edge from the lower-indexed to the higher-indexed vertex. |
| **`forward_euler_edge_do.m`** | Numerically integrates the `edge_do` dynamics using Forward Euler, given the initial state, final time `T`, step size `dt`, and problem data (`n`, `d`, `D_big`, `D_t_big`, gradient handle, `alpha`, `beta`). Returns the time vector `t` and full state history `Xhist` (one row per time step, matching the `ode45` convention). |
| **`edge_do.m`** | Implements the right-hand side of the continuous-time dynamics: given the stacked state `X = [x; v]`, returns `dXdt = [dx; dv]` per the algorithm equations above. |

**Dependency direction:** `driver.m` → `edge_do_main.m` → {`incidence_matrix.m`, `forward_euler_edge_do.m` → `edge_do.m`}.

## Simulation in MATLAB

Fow now, all results use the network and simulation setup fixed inside `edge_do_main.m`:

- **Number of agents:** $n = 6$
- **State dimension:** $d = 1$
- **Simulation horizon:** $T = 10$ s
- **Step size:** $dt = 5 \times 10^{-4}$ s (Forward Euler)
- **Communication graph (adjacency matrix):**

$$
A = \begin{bmatrix}
0 & 1 & 1 & 0 & 1 & 1\\
1 & 0 & 0 & 1 & 0 & 0\\
1 & 0 & 0 & 0 & 1 & 0\\
0 & 1 & 0 & 0 & 1 & 0\\
1 & 0 & 1 & 1 & 0 & 1\\
1 & 0 & 0 & 0 & 1 & 0
\end{bmatrix}
$$

- **Initial conditions:**

$$
x_0 = \begin{bmatrix}2\\1\\0.5\\-0.5\\-1\\-2\end{bmatrix}, \qquad v_0 = \mathbf{0}_6
$$

- **Cost functions:**

$$
\begin{aligned}
f_1(x) &= \sin(x), & \nabla f_1(x) &= \cos(x)\\
f_2(x) &= \cos(x), & \nabla f_2(x) &= -\sin(x)\\
f_3(x) &= e^{0.1x}, & \nabla f_3(x) &= 0.1e^{0.1x}\\
f_4(x) &= (x-4)^4, & \nabla f_4(x) &= 4(x-4)^3\\
f_5(x) &= (x+3)^2, & \nabla f_5(x) &= 2(x+3)\\
f_6(x) &= x^2, & \nabla f_6(x) &= 2x
\end{aligned}
$$

### Usage and Dependencies:

Dependencies: MATLAB R2020a or later (no additional toolboxes required)

Usage:
 - Add all .m files to the MATLAB path (or run from the directory containing them).
 - Run a single experiment directly:
```matlab
edge_do_main(<alpha>, <beta>, 'output_filename.pdf')
```

This simulates the 6-agent network for $T = 10$ s at step size $dt = 5\times10^{-4}$ s with the given $(\alpha, \beta)$, and saves the resulting 3-panel figure (agent states, total cost, summed gradient) as a vector PDF.

Alternatively, run driver.m to reproduce the figures shown in this README:
```matlab
driver
```
### Simulation Results

![Figure 1.1: Simulation results for $\alpha = 2$ and $\beta = 1$.](1.1.pdf)

![Figure 1.2: Simulation results for $\alpha = 2$ and $\beta = 2$.](1.2.pdf)

![Figure 1.3: Simulation results for $\alpha = 2$ and $\beta = 8$.](1.3.pdf)


