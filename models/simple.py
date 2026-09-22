import numpy as np
from dataclasses import dataclass
from enum import Enum

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
    m_dot: float    # Flow rate

@dataclass(frozen=True)
class ComponentResults:
    outlet: FlowState
    omega: float = 0.0
    delta_h0: float = 0.0 # Change in total enthalpy
    Power: float = 0.0
    Torque: float = 0.0

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
    tau_load: float
    I: float
    r_1: float = None
    r_2: float = None
    r_m: float = None   # Mean radius

class MachineType(Enum):
    COMPRESSOR = "compressor"
    TURBINE = "turbine"

class FlowGeometry(Enum):
    AXIAL = "axial"
    RADIAL = "radial"
    MIXED = "mixed"

@dataclass
class Machine:
    machine_type: MachineType
    flow_geometry: FlowGeometry

@dataclass
class TimeParams:
    delta_t: float
    n_steps: float