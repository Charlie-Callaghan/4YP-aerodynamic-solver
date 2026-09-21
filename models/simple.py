import numpy as np
from dataclasses import dataclass

@dataclass(frozen=True)
class FlowState:
    P: float  # Static pressure
    T: float  # Static temperature
    rho: float  # Density
    P0: float   # Stagnation pressure
    T0: float   # Stagnation temperature
    c_x: float  # Absolute axial/meridional velocity
    c_theta: float  # Absolute whirl velocity
    M: float    # Mach number
    alpha: float    # Inlet angle
    beta: float     # Exit angle

@dataclass
class RotorResults:
    c_theta: float
    delta_h0: float # Change in total enthalpy
    P0: float
    T0: float

@dataclass
class StatorResults:
    c_theta: float
    P0: float
    T0: float

@dataclass(frozen=True)
class IdealGas:
    gamma: float
    R: float

    @property
    def c_p(self):  # Specific heat capacity at constant pressure
        return self.gamma * self.R/(self.gamma - 1)

    @property
    def c_v(self):  # Specific heat capacity at constant volume
        return self.c_p - self.R

@dataclass(frozen=True)
class BladeParams:
    alpha: float  # Inlet angle
    beta: float  # Exit angle
    c: float    # Chord length
    s: float    # Blade Pitch
    A: float    # Flow path area
    h: float    # Blade height
    N: float    # Rotational speed
    r_1: float = None
    r_2: float = None
    r_m: float = None   # Mean radius
