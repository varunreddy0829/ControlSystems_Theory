# Model Predictive Control (MPC) — Study Summary

## 1. The Big Idea

MPC is a control strategy that uses a **model of the system** to **predict future behavior** and **optimizes control inputs** over a finite horizon — all while respecting **constraints**.

> **One-sentence summary:** MPC looks ahead, plans the best sequence of moves, applies only the first one, then re-plans using fresh measurements.

---

## 2. The Four Pillars of MPC

### 🔮 Prediction (Model)
- Uses a mathematical model (e.g., state-space: $x_{k+1} = Ax_k + Bu_k$)
- "Unrolls" the dynamics to predict states over a horizon of $N$ steps
- Each future state depends on current state $x_k$ and the input sequence

### 🎯 Optimization (Cost Function)
- Defines **what "good" means** via a cost function to minimize:

$$J = \sum_{i=0}^{N-1} \left( \|x_{k+i} - x_{ref}\|^2_Q + \|u_{k+i}\|^2_R \right)$$

- **Tracking error:** How far from the desired state (weighted by $Q$)
- **Control effort:** How aggressive the inputs are (weighted by $R$)

### 🚧 Constraints
- MPC explicitly includes physical limits in the optimization:
  - Input constraints: $u_{min} \leq u \leq u_{max}$
  - State constraints: $x_{min} \leq x \leq x_{max}$
- **Key advantage over LQR:** MPC *plans within limits*, rather than ignoring them

### 🔄 Receding Horizon
- At each time step:
  1. Solve optimization over horizon $N$
  2. Apply **only the first input** $u_k$
  3. Measure the new state
  4. Shift horizon forward and **repeat**
- Provides robustness to disturbances and model errors

---

## 3. MPC vs. LQR — Key Differences

| Aspect | LQR | MPC |
|--------|-----|-----|
| Cost function | Quadratic | Quadratic |
| Horizon | Infinite | Finite (receding) |
| Constraints | ❌ Cannot handle | ✅ Built-in |
| Computation | Offline (Riccati) | Online (every step) |
| Actuator saturation | Causes mismatch issues | Handled naturally |

**LQR's problem:** If it commands an input beyond actuator limits, saturation occurs but LQR doesn't know — leading to degraded performance or instability.

**MPC's solution:** Constraints are part of the optimization, so it never plans infeasible inputs.

---

## 4. Practical Considerations

### Computational Cost
- MPC solves an optimization problem **at every time step**
- Challenge for fast systems (robotics, drones, high-speed vehicles)

### What Makes MPC Feasible?
| Strategy | Effect |
|----------|--------|
| Shorter horizon $N$ | Smaller problem |
| Efficient QP solvers | Faster solve times |
| Explicit MPC | Pre-computed lookup tables |
| Linear models | Convex QP (easier than nonlinear) |
| Better hardware | More computation per second |

### Where MPC Excels
- Slower processes: chemical plants, HVAC, refineries
- Systems with important constraints
- Reference tracking with look-ahead capability

---

## 5. Conceptual Analogy — Driving a Car

| Driving | MPC Equivalent |
|---------|----------------|
| You see the road ahead | Prediction horizon |
| You know how your car steers | System model |
| You plan smooth steering over next few seconds | Optimization over inputs |
| Steering wheel has a max angle | Input constraints |
| You re-adjust constantly based on where you are | Receding horizon + feedback |

---

## 6. Key Equations at a Glance

**State prediction (unrolled dynamics):**
$$x_{k+1} = Ax_k + Bu_k$$
$$x_{k+2} = A^2x_k + ABu_k + Bu_{k+1}$$

**Optimization problem:**
$$\min_{u_k, \ldots, u_{k+N-1}} \sum_{i=0}^{N-1} \left( \|x_{k+i} - x_{ref}\|^2_Q + \|u_{k+i}\|^2_R \right)$$

**Subject to:**
- Dynamics: $x_{k+i+1} = Ax_{k+i} + Bu_{k+i}$
- Input bounds: $u_{min} \leq u_{k+i} \leq u_{max}$
- State bounds: $x_{min} \leq x_{k+i} \leq x_{max}$