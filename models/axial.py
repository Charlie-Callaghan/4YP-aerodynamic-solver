from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class BladeParams:
    beta_1m: float  # Inlet angle
    beta_2m: float  # Exit angle
    c: float    # Chord length
    s: float    # Blade Pitch
    r_m: float  # Mean radius
    A: float    # Flow path area
    h: float    # Blade height
    N: float    # Rotational speed

    def U(self) -> float:
        return self.N * self.r_m

@dataclass(frozen=True)
class OperParams:
    P_01: float     # Total pressure
    T_01: float     # Total temperature
    P_ex: float     # Back pressure
    m_dot: float    # Mass flow rate

@dataclass(frozen=True)
class FlowParams:
    V_z: float      # Axial component of absolute velocity
    V_theta: float  # Swirl component of absolute velocity
    W_z: float      # Axial component of relative velocity
    W_theta: float  # Swirl component of relative velocity
    alpha: float    # Absolute flow angle
    beta: float     # Relative flow angle

@dataclass(frozen=True)
class ThermoParams:
    P: float    # Static pressure
    T: float    # Static temperature
    h: float    # Static enthalpy
    h_0: float  # Total enthalpy
    rho: float  # Density
    s: float    # Entropy

