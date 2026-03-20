"""
Spring-Mass-Damper System

A classic second-order linear system consisting of a mass connected 
to a spring and damper in parallel.

Dynamics:
    m * x_ddot + c * x_dot + k * x = F + w
    
Where:
    x     : position (displacement from equilibrium)
    x_dot : velocity
    m     : mass
    c     : damping coefficient
    k     : spring stiffness
    F     : control force (input)
    w     : disturbance force

State-space form with state = [position, velocity]:
    
    | x_dot     |   |  0      1   | | x     |   |  0  |       |  0  |
    |           | = |             | |       | + |     | * F + |     | * w
    | x_ddot    |   | -k/m  -c/m  | | x_dot |   | 1/m |       | 1/m |

ASCII Diagram:
    
          +---> F (control force)
          |
    ======|======
    |     v     |
    |  +-----+  |
    |  |  m  |  |----/\/\/\---- (spring, k)
    |  +-----+  |
    |     |     |----[dashpot]- (damper, c)
    |     v     |
    ======|======
          |
          +---> w (disturbance)
"""

import numpy as np
from .base_system import BaseSystem


class SpringMassDamper(BaseSystem):
    """
    Spring-Mass-Damper system implementation.
    """
    
    def __init__(self, mass: float, damping: float, stiffness: float):
        """
        Initialize spring-mass-damper system.
        
        Args:
            mass: Mass of the object [kg]
            damping: Damping coefficient [N*s/m]
            stiffness: Spring stiffness [N/m]
        """
        super().__init__(n_states=2, n_inputs=1)
        
        # Store physical parameters
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        
        # Validate parameters
        if mass <= 0:
            raise ValueError("Mass must be positive")
        if damping < 0:
            raise ValueError("Damping must be non-negative")
        if stiffness < 0:
            raise ValueError("Stiffness must be non-negative")
    
    def get_continuous_matrices(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Return continuous-time state-space matrices.
        
        Returns:
            A_c: State matrix (2, 2)
            B_c: Input matrix (2, 1)
            B_w_c: Disturbance matrix (2, 1)
        """
        m = self.mass
        c = self.damping
        k = self.stiffness
        
        # State matrix
        A_c = np.array([
            [0.0,       1.0     ],
            [-k/m,     -c/m     ]
        ])
        
        # Input matrix (force applied to mass)
        B_c = np.array([
            [0.0  ],
            [1.0/m]
        ])
        
        # Disturbance matrix (disturbance force on mass)
        G_c = np.array([
            [0.0  ],
            [1.0/m]
        ])
        
        return A_c, B_c, G_c
    
    def get_state_names(self) -> list[str]:
        """Return human-readable names for each state."""
        return ["position", "velocity"]
    
    def get_input_names(self) -> list[str]:
        """Return human-readable names for each input."""
        return ["force"]
    
    def get_natural_frequency(self) -> float:
        """
        Calculate the natural frequency of the undamped system.
        
        Returns:
            omega_n: Natural frequency [rad/s]
        """
        return np.sqrt(self.stiffness / self.mass)
    
    def get_damping_ratio(self) -> float:
        """
        Calculate the damping ratio.
        
        Returns:
            zeta: Damping ratio (dimensionless)
                  zeta < 1: underdamped
                  zeta = 1: critically damped
                  zeta > 1: overdamped
        """
        omega_n = self.get_natural_frequency()
        return self.damping / (2 * self.mass * omega_n)
    
    def __repr__(self) -> str:
        """String representation of the system."""
        return (
            f"SpringMassDamper("
            f"m={self.mass}, c={self.damping}, k={self.stiffness}, "
            f"omega_n={self.get_natural_frequency():.3f} rad/s, "
            f"zeta={self.get_damping_ratio():.3f})"
        )