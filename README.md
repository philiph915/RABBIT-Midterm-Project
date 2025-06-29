# Motion Planning Optimization for the Planar 5-Link Biped (RABBIT)

**Philip Hancock**  
ME6984  
4/11/2019  

---

## Background and Motivation

This project presents the results of a motion planning optimization for the planar 5-link biped **RABBIT**. This robot uses feedback linearization to asymptotically drive virtual constraint output equations to zero.

- **Stance phase**: 5 degrees of freedom, 4 actuators → 1 degree of underactuation  
- **Flight phase**: 7 degrees of freedom, 4 actuators

There are 4 actuated joints, so 4 virtual constraint outputs.

---

## Hybrid System Dynamics

The hybrid dynamics of the system are:

$$
\Sigma = \begin{cases}
\dot{x} = f(x) + g(x)u, & x^- \notin S \\
x^+ = \Delta(x^-), & x^- \in S
\end{cases}
$$

Where:

$$
f(x) = -D^{-1}(q)\left(C(q, \dot{q})\dot{q} + G(q)\right)
$$

$$
g(x) = D^{-1}(q)B
$$

- $D(q)$: mass-inertia matrix (N×N)  
- $C(q, \dot{q})$: Coriolis and centrifugal terms (N×N)  
- $G(q)$: gravity vector (N×1)  
- $B$: input matrix (N×M)  
- $\Delta$: impact map  
- $x^+, q^+$: post-impact state  
- $x^-, q^-$: pre-impact state

**Switching manifold**:

$$
S := \{(x, y) \in \mathbb{R}^2 \mid x > 0,\ y = 0\}
$$

---

## Impact Dynamics

The impact dynamics are given by the following system of equations:

$$
\begin{bmatrix} 
D_e(q_e) & -J_e^T(q_e) \\ J_e(q_e) & 0_{2\times2} 
\end{bmatrix} 
\begin{bmatrix} 
\dot{q}_e^+ \\ \delta F 
\end{bmatrix} = \begin{bmatrix} D_e(q_e)\dot{q}_e^- \\ 0_{2\times1} \end{bmatrix}
$$

Where:

- $D_e$: extended mass-inertia matrix  
- $J_e$: Jacobian of the swing foot  
- $\delta F$: impulsive ground reaction force

---

## Phasing Variable

A strictly increasing **phasing variable** $\theta(q)$ is defined as the angle between the virtual leg and the negative x-axis:

$$
\theta = c_1 q + c_0
$$

Where:

$$
c_1 =
\begin{bmatrix}
-1 & 0.5 & 0 & -1
\end{bmatrix},
\quad
c_0 = \frac{3\pi}{2}
$$

---

## Optimization Problem

The goal is to compute the trajectory of the **controlled variables** $h_d(\theta)$ that minimizes energy while satisfying feasibility and periodicity constraints.

### Decision Variables

- $x_0$: state vector before impact  
- $\alpha_{mid}$: Bezier coefficients (middle only)  

Other coefficients are derived from boundary conditions.

### Cost Function

Minimize average squared torque over a step:

$$
J(x_0, \alpha) := \frac{1}{T} \int_0^T \|u(t)\|^2 dt
$$

---

### Equality Constraints

Enforce periodicity:

$$
\phi\left[T(\Delta(x_0), \alpha),\ \Delta(x_0),\ \alpha\right] - x_0 = 0
$$

---

### Inequality Constraints

Ensure physical feasibility:

- $\theta(q)$ strictly increasing  
- Joint, velocity, and torque bounds  
- Step length $\geq 0.2$ m  
- Average speed $\geq 0.8$ m/s  

#### Ground Reaction Force:

- $F_y > 0$  
- $|F_x| < \mu F_y$  
- Same for impulse force

#### Swing Foot:

- Start $x < -0.1$ m  
- End $x > 0.1$ m  
- Max height $\geq 0.02$ m  
- Must not go below $y = 0$  
- Must be descending at impact  
- Step time $\geq 0.2$ s

---

## Optimization Results

**Cost:**

$$
J = 4793.99
$$

**Initial State $x^-$:**

$$
q^- =
\begin{bmatrix}
3.3075 \\ 3.7886 \\ 0.3504 \\ 0.1015 \\ -0.2709
\end{bmatrix},
\quad
\dot{q}^- =
\begin{bmatrix}
-0.4286 \\ 0.0589 \\ 0.0014 \\ 0.7109 \\ -0.7630
\end{bmatrix}
$$

**Bezier Coefficients $\alpha_{mid}$:**

$$
\alpha_{mid} =
\begin{bmatrix}
3.5765 \\ 3.4834 \\ 0.0477 \\ 0.3367 \\
3.4044 \\ 3.7020 \\ 0.1331 \\ 0.1796
\end{bmatrix}
$$

---

## Closed-Loop Simulations

See the full write up PDF file for closed-loop simulation results for the following:

### Optimal Initial Conditions

- 5-step periodic simulation  
- Stable trajectory and torques

### Off-Optimal Initial Conditions

- 20-step run from perturbed state  
- Converges back to optimal gait  
- Demonstrates robustness

---


```
