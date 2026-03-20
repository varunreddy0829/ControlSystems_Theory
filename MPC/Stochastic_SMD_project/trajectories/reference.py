"""
Reference trajectory generators.

Each trajectory class generates position and velocity references
for the controller to track.
"""

from abc import ABC, abstractmethod
import numpy as np


class ReferenceTrajectory(ABC):
    """
    Abstract base class for reference trajectories.
    
    All trajectories must provide both position and velocity references,
    since our state is [position, velocity].
    """
    
    def __init__(self, n_states: int = 2):
        """
        Initialize trajectory.
        
        Args:
            n_states: Number of states in the reference
        """
        self.n_states = n_states
    
    @abstractmethod
    def get_reference(self, t: float) -> np.ndarray:
        """
        Get reference state at time t.
        
        Args:
            t: Time [s]
            
        Returns:
            x_ref: Reference state (n_states, 1)
        """
        pass
    
    def get_reference_sequence(self, t_start: float, dt: float, n_steps: int) -> np.ndarray:
        """
        Get a sequence of reference states.
        
        Useful for MPC which needs the reference over the prediction horizon.
        
        Args:
            t_start: Starting time [s]
            dt: Time step [s]
            n_steps: Number of steps
            
        Returns:
            X_ref: Reference states (n_states, n_steps)
        """
        X_ref = np.zeros((self.n_states, n_steps))
        for i in range(n_steps):
            t = t_start + i * dt
            X_ref[:, i:i+1] = self.get_reference(t)
        return X_ref
    
    def get_full_trajectory(self, t_array: np.ndarray) -> np.ndarray:
        """
        Get reference states for an array of times.
        
        Args:
            t_array: Array of time values (n_times,)
            
        Returns:
            X_ref: Reference states (n_states, n_times)
        """
        n_times = len(t_array)
        X_ref = np.zeros((self.n_states, n_times))
        for i, t in enumerate(t_array):
            X_ref[:, i:i+1] = self.get_reference(t)
        return X_ref


class SinusoidalTrajectory(ReferenceTrajectory):
    """
    Sinusoidal reference trajectory.
    
    Position: x(t) = amplitude * sin(2 * pi * frequency * t + phase) + offset
    Velocity: v(t) = dx/dt (analytical derivative)
    """
    
    def __init__(
        self, 
        amplitude: float = 1.0, 
        frequency: float = 0.5, 
        phase: float = 0.0,
        offset: float = 0.0
    ):
        """
        Initialize sinusoidal trajectory.
        
        Args:
            amplitude: Amplitude of oscillation [m]
            frequency: Frequency [Hz]
            phase: Phase offset [rad]
            offset: DC offset for position [m]
        """
        super().__init__(n_states=2)
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase
        self.offset = offset
        
        # Angular frequency
        self.omega = 2 * np.pi * frequency
    
    def get_reference(self, t: float) -> np.ndarray:
        """Get reference state at time t."""
        # Position: A * sin(omega * t + phi) + offset
        position = self.amplitude * np.sin(self.omega * t + self.phase) + self.offset
        
        # Velocity: d/dt of position = A * omega * cos(omega * t + phi)
        velocity = self.amplitude * self.omega * np.cos(self.omega * t + self.phase)
        
        return np.array([[position], [velocity]])
    
    def __repr__(self) -> str:
        return (
            f"SinusoidalTrajectory("
            f"A={self.amplitude}, f={self.frequency}Hz, "
            f"phase={self.phase}, offset={self.offset})"
        )


class StepTrajectory(ReferenceTrajectory):
    """
    Step reference trajectory.
    
    Position jumps from initial_value to final_value at step_time.
    Velocity reference is zero (we want to settle at the target).
    """
    
    def __init__(
        self,
        initial_value: float = 0.0,
        final_value: float = 1.0,
        step_time: float = 0.0
    ):
        """
        Initialize step trajectory.
        
        Args:
            initial_value: Position before step [m]
            final_value: Position after step [m]
            step_time: Time at which step occurs [s]
        """
        super().__init__(n_states=2)
        self.initial_value = initial_value
        self.final_value = final_value
        self.step_time = step_time
    
    def get_reference(self, t: float) -> np.ndarray:
        """Get reference state at time t."""
        if t < self.step_time:
            position = self.initial_value
        else:
            position = self.final_value
        
        # Velocity reference is zero (we want to be stationary at target)
        velocity = 0.0
        
        return np.array([[position], [velocity]])
    
    def __repr__(self) -> str:
        return (
            f"StepTrajectory("
            f"{self.initial_value} -> {self.final_value} at t={self.step_time}s)"
        )


class ConstantTrajectory(ReferenceTrajectory):
    """
    Constant reference trajectory.
    
    Maintains a fixed position with zero velocity.
    Useful for regulation (setpoint) problems.
    """
    
    def __init__(self, position: float = 0.0):
        """
        Initialize constant trajectory.
        
        Args:
            position: Desired constant position [m]
        """
        super().__init__(n_states=2)
        self.position = position
    
    def get_reference(self, t: float) -> np.ndarray:
        """Get reference state at time t."""
        return np.array([[self.position], [0.0]])
    
    def __repr__(self) -> str:
        return f"ConstantTrajectory(position={self.position})"