import numpy as np
from physics.reduced_order import get_omega, get_U

# Inputs
P_01 = 1    # Total pressure at rotor inlet
T_01 = 1    # Total temperature at rotor inlet
N = 1       # Shaft speed in rpm
m_dot = 1   # Mass flow rate

# Design params
r_1 = 1 # Radius at impellor inlet
r_2 = 2 # Radius at impellor exit
A_1 = 1 # Area at impellor inlet
A_2 = 1 # Area at impellor exit

alpha1 = 0  # Angle of inlet flow velocity
beta2 = np.pi/3 # Angle of flow velocity at impellor exit
rho = 1 # Density (assumed constant)
c_p = 1 # Specific heat capacity at constant pressure
gamma = 1   # c_p/c_v

losses = 0  # Losses coef

eta_rotor = 1   # Efficiency terms
eta_stator = 1

# shaft speed
omega = 2 * np.pi * N / 60
U_1 = omega * r_1
U_2 = omega * r_2

# Continuity
c_m1 = m_dot / (rho * A_1)
c_m2 = m_dot / (rho * A_1)

# Impellor inlet
c_theta1 = c_m1 * np.tan(alpha1)
w_theta1 = c_theta1 - U_1


# Impellor outlet
c_theta2 = U_2 + c_m2 * np.tan(beta2)
c_2 = np.sqrt(c_m2**2 + c_theta2**2)

# Euler equation
delta_h0 = U_2 * c_theta2 - U_1 * c_theta1

# Stagnation temperature increase
T_02 = T_01 + delta_h0 / c_p

# Stagnation pressure increase and pressure ratio
PR = (T_02 / T_01)**(gamma/(gamma-1))
P_02 = P_01 * PR

# Power input
P = m_dot * delta_h0

