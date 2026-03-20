"""
Abstract base class for dynamical systems.

All systems should inherit from this class and implement the required methods.
This ensures a consistent interface across different system types.
"""

from abc import ABC, abstractmethod
import numpy as np
from scipy.linalg import expm, inv


class BaseSystem(ABC):
    """
    Abstract base class for continuous-time dynamical systems.
    
    System form:
        dx/dt = A_c @ x + B_c @ u + G_c @ w
        
    Where:
        x: state vector (n_states, 1)
        u: control input (n_inputs, 1)
        w: disturbance (scalar or vector)
        
    Notation:
        A: state matrix
        B: input matrix
        G: disturbance matrix
        
        Subscript _c: continuous-time
        Subscript _d: discrete-time
    """
    
    def __init__(self, n_states: int, n_inputs: int):
        """
        Initialize system dimensions.
        
        Args:
            n_states: Number of states
            n_inputs: Number of control inputs
        """
        self.n_states = n_states
        self.n_inputs = n_inputs
        
        # Discrete-time matrices (populated after discretization)
        self.A_d = None
        self.B_d = None
        self.G_d = None
        self.dt = None
    
    @abstractmethod
    def get_continuous_matrices(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Return continuous-time state-space matrices.
        
        Returns:
            A_c: State matrix (n_states, n_states)
            B_c: Input matrix (n_states, n_inputs)
            G_c: Disturbance matrix (n_states, n_disturbances)
        """
        pass
    
    def discretize(self, dt: float, method: str = "zoh") -> None:
        """
        Convert continuous-time system to discrete-time.
        
        Args:
            dt: Time step for discretization
            method: Discretization method ("euler" or "zoh")
        """
        self.dt = dt
        A_c, B_c, G_c = self.get_continuous_matrices()
        
        if method == "euler":
            # Simple Euler discretization: x_{k+1} = (I + dt*A)x_k + dt*B*u_k
            self.A_d = np.eye(self.n_states) + dt * A_c
            self.B_d = dt * B_c
            self.G_d = dt * G_c
            
        elif method == "zoh":
            # Zero-order hold (exact discretization for linear systems)
            # A_d = e^(A_c * dt)
            self.A_d = expm(A_c * dt)
            
            # B_d = A_c^(-1) @ (A_d - I) @ B_c
            # For singular A_c, use numerical integration or approximation
            try:
                A_c_inv = inv(A_c)
                self.B_d = A_c_inv @ (self.A_d - np.eye(self.n_states)) @ B_c
                self.G_d = A_c_inv @ (self.A_d - np.eye(self.n_states)) @ G_c
            except np.linalg.LinAlgError:
                # A_c is singular, fall back to approximation
                self.B_d = dt * B_c
                self.G_d = dt * G_c
        else:
            raise ValueError(f"Unknown discretization method: {method}")
    
    def get_discrete_matrices(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Return discrete-time state-space matrices.
        
        Returns:
            A_d: Discrete state matrix (n_states, n_states)
            B_d: Discrete input matrix (n_states, n_inputs)
            G_d: Discrete disturbance matrix (n_states, n_disturbances)
            
        Raises:
            ValueError: If system has not been discretized yet
        """
        if self.A_d is None:
            raise ValueError("System not discretized. Call discretize(dt) first.")
        return self.A_d, self.B_d, self.G_d
    
    def step(self, x: np.ndarray, u: float, w: float = 0.0) -> np.ndarray:
        """
        Simulate one discrete time step.
        
        Args:
            x: Current state (n_states, 1)
            u: Control input (scalar)
            w: Disturbance (scalar)
            
        Returns:
            x_next: Next state (n_states, 1)
        """
        if self.A_d is None:
            raise ValueError("System not discretized. Call discretize(dt) first.")
        
        x_next = self.A_d @ x + self.B_d * u + self.G_d * w
        return x_next
    
    @abstractmethod
    def get_state_names(self) -> list[str]:
        """Return human-readable names for each state."""
        pass
    
    @abstractmethod
    def get_input_names(self) -> list[str]:
        """Return human-readable names for each input."""
        pass