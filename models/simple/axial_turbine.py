import numpy as np

# Inputs
P_01 = 1    # Total pressure at rotor inlet
T_01 = 1    # Total temperature at rotor inlet
N = 1       # Shaft speed in rpm
alpha1 = 0  # Angle of inlet flow velocity
alpha2 = np.pi/4
m_dot = 1   # Mass flow rate
h_01 = 1

# Design params
r_m = 1 # Mean radius 
A = 1   # Annulus 
beta2 = np.pi/3 # Angle of relative velocity at stator exit
beta3 = np.pi/4 # Angle of relative velocity at rotor exit
rho = 1 # Density
c_p = 1 # Specific heat capacity at constant pressure
gamma = 1   # c_p/c_v

losses = 0  # Losses coef

eta_rotor = 1   # Efficiency terms
eta_stator = 1

# Shaft speed
omega = 2 * np.pi * N / 60
U = omega * r_m

# Continuity
c_z = m_dot / (rho * A)

# Velocity triangles
c_theta1 = c_z * np.tan(alpha1)
c_theta2 = c_z * np.tan(alpha2) # Stator

c_theta3 = U + c_z * np.tan(beta3)

# Euler equation

delta_h0 = U * (c_theta3 - c_theta2)
h_02 = h_01
h_03 = h_02 + delta_h0

W_s = m_dot * delta_h0  # SHaft work

# Thermogynamics
T_02 = T_01
T_03 = T_02 + delta_h0 / c_p