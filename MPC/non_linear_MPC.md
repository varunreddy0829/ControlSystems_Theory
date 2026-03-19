# Nonlinear Model Predictive Control (NMPC)

## 1. The Problem

In many real systems, the dynamics are **nonlinear**:

$$x_{k+1} = f(x_k, u_k)$$

where $f$ is a nonlinear function (e.g., includes trigonometric terms, products of states, etc.).

**Key challenge:** We can no longer formulate the problem as a Quadratic Program (QP).

---

## 2. Why Nonlinearity Breaks the QP Structure

### Linear Case (Review)
With $x_{k+1} = Ax_k + Bu_k$, we can unroll:

$$x_{k+2} = A^2x_k + ABu_k + Bu_{k+1}$$

Future states are **linear functions** of inputs → QP formulation → **convex** → global optimum guaranteed.

### Nonlinear Case
With $x_{k+1} = f(x_k, u_k)$, we can still unroll:

$$x_{k+2} = f(f(x_k, u_k), u_{k+1})$$

But now future states are **nonlinear functions** of inputs → **Nonlinear Program (NLP)** → potentially **non-convex** → no global optimum guarantee.

---

## 3. Convex vs Non-Convex Optimization

### QP: Convex Landscape
- Cost function looks like a **bowl**
- Single **global minimum**
- Solver always finds it
- Fast and reliable

### NLP: Non-Convex Landscape
- Cost function looks like a **mountain range**
- Multiple **local minima**
- Solver may get stuck in suboptimal valley
- Solution depends on **initial guess**
- Slower and less reliable

---

## 4. Three Approaches to MPC

| Type | Model | Optimization | Global Optimum? | Speed |
|------|-------|--------------|-----------------|-------|
| **Linear MPC** | Linear (or linearized once at fixed point) | QP | ✅ Yes | Fast |
| **RTI / Linearized MPC** | Re-linearize at each time step | QP | ✅ Yes (each QP) | Medium |
| **Full Nonlinear MPC** | True nonlinear model | NLP | ❌ No guarantee | Slow |

---

## 5. Linearization Approach (RTI)

### The Idea
Linearize $f(x, u)$ around the current operating point $(x_k, u_k)$:

$$x_{k+1} \approx f(x_k, u_k) + A_k(x - x_k) + B_k(u - u_k)$$

where:
- $A_k = \frac{\partial f}{\partial x}\big|_{x_k, u_k}$ (Jacobian w.r.t. state)
- $B_k = \frac{\partial f}{\partial u}\big|_{x_k, u_k}$ (Jacobian w.r.t. input)

### Real-Time Iteration (RTI) Algorithm
1. Linearize model at current state
2. Solve the resulting QP
3. Apply **only the first input**
4. Measure new state
5. **Repeat** (re-linearize at new point)

### Why RTI Works
- Linearization is only accurate **locally** (near the linearization point)
- But we only apply the **first input** (receding horizon!)
- By the time we need step 2, we've re-linearized with fresh information
- Errors in later predictions don't matter — we never use them

### Limitation
If the system moves very far in one step, or the nonlinearity is very strong, the linear approximation may be too inaccurate even for one step.

---

## 6. Full Nonlinear MPC

### The Idea
Don't linearize — solve the true nonlinear problem directly:

$$\min_{u_k, \ldots, u_{k+N-1}} \sum_{i=0}^{N-1} \left( \|x_{k+i} - x_{ref}\|^2_Q + \|u_{k+i}\|^2_R \right)$$

**Subject to:**
- $x_{k+i+1} = f(x_{k+i}, u_{k+i})$ ← nonlinear equality constraints
- $u_{min} \leq u_{k+i} \leq u_{max}$
- $x_{min} \leq x_{k+i} \leq x_{max}$

### NLP Solvers
Common solvers for nonlinear MPC:
- **IPOPT** — Interior Point Optimizer
- **SNOPT** — Sparse Nonlinear Optimizer
- **CasADi** — Framework for automatic differentiation + interfaces to solvers
- **ACADO** — Toolkit for automatic control and dynamic optimization

### How NLP Solvers Work (Simplified)
1. Start from an initial guess
2. Iteratively improve the solution
3. Stop when converged (or time limit reached)

Common methods:
- **Interior Point Methods** — handle constraints via barrier functions
- **SQP (Sequential Quadratic Programming)** — solve a sequence of QP approximations

---

## 7. Dealing with Local Minima

Since we can't guarantee global optimum, we use practical strategies:

| Strategy | Description |
|----------|-------------|
| **Good initial guess** | Use previous MPC solution, shifted by one step |
| **Warm starting** | Give solver information from last solve |
| **Problem design** | Shape cost/constraints to reduce bad local minima |
| **Multiple starts** | Try several initial guesses, pick best (expensive) |
| **Accept "good enough"** | A feasible local minimum is often acceptable |

### The Practical Reality
In real-time control, what matters is:
- ✅ Solution is **feasible** (satisfies constraints)
- ✅ Solution is **good enough** (reasonable performance)
- ✅ Solution arrives **in time** (within the control loop)

A local minimum that keeps your system stable and performing well is perfectly acceptable!

---

## 8. When to Use Each Approach

### Linear MPC
- System is naturally linear
- Operating near a fixed equilibrium point
- Need maximum speed

### RTI / Linearized MPC
- Mildly nonlinear systems
- System doesn't move too far per time step
- Good balance of speed and accuracy

### Full Nonlinear MPC
- Highly nonlinear systems
- Large operating range (far from any single linearization point)
- Accuracy is critical and computation time is available

**Example:** 
- Pendulum **balancing** at the top → RTI probably fine (small motions)
- Pendulum **swing-up** from bottom to top → Full NMPC needed (large nonlinear motion)

---

## 9. Key Takeaways

1. **Nonlinearity breaks convexity** — NLP instead of QP
2. **Local minima are unavoidable** — but often acceptable in practice
3. **RTI is a clever compromise** — re-linearize each step, solve QPs, still fast
4. **Full NMPC is most accurate** — but computationally expensive
5. **Initial guess matters** — warm-start from previous solution
6. **"Good enough" is the goal** — feasible + reasonable + fast beats optimal + late