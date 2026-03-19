# MPC Control Comparison Project

A modular Python implementation comparing **Model Predictive Control (MPC)**, **LQR**, and **PID** controllers on a spring-mass-damper system with trajectory tracking, tight input constraints, and stochastic disturbances.

---

## Project Goals

1. **Understand MPC from the ground up** by implementing it from scratch
2. **Compare controller performance** under realistic conditions (constraints + noise)
3. **Build a modular, extensible codebase** that can be reused for other systems

---

## Project Structure

```
mpc_project/
│
├── README.md                   # This file
├── requirements.txt            # Python dependencies
├── config.py                   # All tunable parameters in one place
│
├── systems/                    # Dynamical system models
│   ├── __init__.py
│   ├── base_system.py          # Abstract base class for systems
│   └── spring_mass_damper.py   # Spring-mass-damper implementation
│
├── controllers/                # Controller implementations
│   ├── __init__.py
│   ├── base_controller.py      # Abstract base class for controllers
│   ├── pid_controller.py       # PID controller
│   ├── lqr_controller.py       # LQR controller
│   └── mpc_controller.py       # MPC controller
│
├── simulation/                 # Simulation engine
│   ├── __init__.py
│   └── simulator.py            # Runs closed-loop simulations
│
├── utils/                      # Utility functions
│   ├── __init__.py
│   ├── metrics.py              # Performance metrics (RMSE, etc.)
│   └── plotting.py             # Visualization functions
│
├── trajectories/               # Reference trajectory generators
│   ├── __init__.py
│   └── reference.py            # Sinusoidal, step, custom trajectories
│
└── main.py                     # Entry point: run experiments
```

---

## Modules Overview

### `config.py`
Central configuration file containing:
- System parameters (mass, damping, stiffness)
- Simulation parameters (time step, duration)
- Constraint values (force limits)
- Disturbance parameters (noise standard deviation)
- Controller tuning parameters (PID gains, LQR/MPC weights, horizon)

### `systems/`
Defines dynamical systems with a common interface:
- `get_continuous_matrices()` → Returns $(A_c, B_c)$
- `get_discrete_matrices(dt)` → Returns $(A_d, B_d)$
- `step(x, u, w)` → Simulates one time step with disturbance

### `controllers/`
Each controller implements a common interface:
- `compute_control(x_current, x_reference)` → Returns control input $u$
- `reset()` → Resets internal states (for PID integral, etc.)

### `simulation/`
The simulation engine that:
- Takes a system + controller + reference trajectory
- Runs the closed-loop simulation
- Applies disturbances and input saturation
- Returns state/input histories

### `trajectories/`
Generates reference trajectories:
- Sinusoidal
- Step response
- Custom (user-defined)

### `utils/`
- **metrics.py**: Computes RMSE, max error, control effort, saturation %
- **plotting.py**: Creates comparison plots

---

## How to Run

```bash
# Install dependencies
pip install -r requirements.txt

# Run the comparison experiment
python main.py
```

---

## Expected Output

1. **Console output**: Performance metrics table comparing all controllers
2. **Plot**: Three-panel figure showing:
   - Position tracking (reference vs actual)
   - Tracking error over time
   - Control input with constraint boundaries

---

## Configuration

All parameters are in `config.py`. Key settings:

```python
# System
MASS = 1.0          # kg
DAMPING = 0.5       # N·s/m
STIFFNESS = 2.0     # N/m

# Constraints
F_MAX = 3.0         # N (tight constraint)
F_MIN = -3.0        # N

# Disturbance
DISTURBANCE_STD = 0.5  # N

# MPC
MPC_HORIZON = 20    # prediction steps
```

---

## Experiments to Try

After running the base comparison:

1. **Vary constraint tightness**: Change `F_MAX` from 3.0 → 10.0 → 1.0
2. **Vary disturbance level**: Change `DISTURBANCE_STD` from 0.5 → 0.0 → 1.5
3. **Vary MPC horizon**: Change `MPC_HORIZON` from 20 → 5 → 50
4. **Different trajectories**: Switch from sinusoidal to step response

---

## Expected Results

Based on theory, we expect:

| Condition | Winner |
|-----------|--------|
| Tight constraints + disturbance | **MPC** (constraint-aware planning) |
| No constraints | LQR ≈ MPC (both optimal) |
| No disturbance + no constraints | All similar (easy problem) |

---

## Future Extensions

- [ ] Nonlinear MPC with CasADi
- [ ] Different systems (pendulum, vehicle model)
- [ ] State constraints (not just input)
- [ ] Obstacle avoidance
- [ ] Real-time visualization

---

## References

- Rawlings, J.B., Mayne, D.Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design*
- Åström, K.J., & Murray, R.M. (2021). *Feedback Systems: An Introduction for Scientists and Engineers*