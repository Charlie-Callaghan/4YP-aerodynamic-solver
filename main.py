import numpy as np

# Inputs
P_01 = 1    # Total pressure at rotor inlet
T_01 = 1    # Total temperature at rotor inlet
N = 1       # Shaft speed in rpm
alpha1 = 0  # Angle of inlet flow velocity
m_dot = 1   # Mass flow rate

# Design params
r_m = 1 # Mean radius 
A = 1   # Annulus 
beta2 = np.pi/3 # Angle of relative velocity at rotor exit
rho = 1 # Density
c_p = 1 # Specific heat capacity at constant pressure
gamma = 1   # c_p/c_v

losses = 0  # Losses coef

eta_rotor = 1   # Efficiency terms
eta_stator = 1

# Continuity

c_z = m_dot / (rho * A)

# Velocity triangle calculations
omega = 2*N*np.pi / 60
U = omega * r_m

c_theta1 = c_z * np.tan(alpha1)
c_theta2 = U - c_z * np.tan(beta2)

c_theta3 = c_theta1 # Assume no change in KE

# Energy Equation
dh0 = U * (c_theta2 - c_theta1)   # Specific work

# Enthalpy difference
rotor_dh = eta_rotor * dh0 + (c_theta1^2 - c_theta2^2)/2
stator_dh = (c_theta2^2 - c_theta3^2)/2 - losses

# Incompressibility condition dp = rho*dh
rotor_dp = rotor_dh * rho
stator_dp = stator_dh * rho

stage_dp = rotor_dp + stator_dp # Pressure difference of stage

# Temperature change
T_02 = T_01 + dh0 / c_p
T_03 = T_02 # Ideal assumption

# Total pressure
P_02 = P_01 * (T_02/T_01)^(gamma/(gamma-1))

