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
    h0: float


@dataclass(frozen=True)
class ComponentResults:
    outlet: FlowState
    state: SystemState
    params: ControlVolume
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

@dataclass(frozen=True)
class Machine:
    machine_type: MachineType
    flow_geometry: FlowGeometry

@dataclass(frozen=True)
class TimeParams:
    delta_t: float
    n_steps: float

@dataclass(frozen=True)
class SystemState:
    V: float
    omega: float
    m: float = 0.0
    E: float = 0.0

@dataclass(frozen=True)
class ControlVolume:
    outlet: FlowState
    m_dot_out: float = 0.0
    m_dot_in: float = 0.0
    h0_in: float = 0.0   # stagnation enthalpy
    h0_out: float = 0.0
    W_s_dot: float = 0.0
    Q_dot: float = 0.0
    V: float = 0.0    # Volume of CV
    K: float = 0.0
    P_d: float = 0.0

@dataclass(frozen=True)
class Derivatives:
    dm_dt1: float
    dm_dt2: float
    dE_dt1: float
    dE_dt2: float
    domega_dt: float