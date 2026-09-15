import numpy as np

# Inputs
P_01 = 1    # Total pressure at rotor inlet
T_01 = 1    # Total temperature at rotor inlet
N = 1       # Shaft speed in rpm
m_dot = 1   # Mass flow rate

# Design params
r_2 = 1 # Radius at impellor inlet
r_3 = 2 # Radius at impellor exit
A_2 = 1 # Area at impellor inlet
A_3 = 1 # Area at impellor exit

alpha1 = 0  # Angle of inlet flow velocity
alpha2 = np.pi/4
beta2 = np.pi/3 # Angle of flow velocity at impellor exit
beta3 = 0
rho = 1 # Density (assumed constant)
c_p = 1 # Specific heat capacity at constant pressure
gamma = 1   # c_p/c_v

losses = 0  # Losses coef

eta_rotor = 1   # Efficiency terms
eta_stator = 1

# Shaft speed
omega = 2 * np.pi * N /60

U_2 = omega * r_2
U_3 = omega * r_3

# Continuity
c_m2 = m_dot / (rho * A_2)
c_m3 = m_dot / (rho * A_3)

# Nozzle
c_theta2 = c_m2 * np.tan(alpha2)

c_2 = np.sqrt(c_m2**2 + c_theta2**2)

# Rotor 
w_theta2 = c_theta2 - U_2
c_theta3 = U_3 + c_m3*np.tan(beta3)

# Eulers equation
delta_h0 = U_3 * c_theta3 - U_2 * c_theta2

# Power
W_s = -m_dot * delta_h0

# Stagnation temperatures
T_02 = T_01
T_03 = T_02 + delta_h0 / c_p