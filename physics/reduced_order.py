import numpy as np
from models.simple import FlowState,BladeParams,IdealGas,RotorResults,StatorResults

# In the following section a few things must be noted:
# - all angles are in radians
# - the term c_x could be meridonial c_m or axial c_z velocity
# - positive theta is the direction of rotor rotation
# - c_theta and w_theta are signed components

# Convert shaft rotational speed from revolutions per minute (rpm)
# to angular velocity, omega, in rad/s.
#
# omega = 2*pi*N / 60
def get_omega(N:float):
    return 2*np.pi*N/60


# Calculate the blade peripheral/tangential velocity at a given radius.
#
# U = r*omega
#
# This is particularly important for radial machines, where the inlet
# and outlet radii may differ and therefore U1 != U2.
def get_U(r:float, omega: float):
    return r * omega


# Calculate the meridional component of absolute velocity using
# conservation of mass (continuity).
#
# m_dot = rho*A*c_m
#
# c_m represents the through-flow velocity. For an axial machine this
# is approximately the axial velocity, while for a radial machine it
# may represent a predominantly radial velocity.
def get_c_m(m_dot: float, rho: float, A: float):
    return m_dot / (rho * A)


# Calculate the magnitude of the absolute flow velocity, c, from its
# meridional and tangential (whirl) components.
#
# c^2 = c_m^2 + c_theta^2
def get_c(c_x: float, c_theta: float):
    return np.sqrt(c_x**2 + c_theta**2)


# Calculate the magnitude of the relative velocity, w, seen by a
# rotating blade.
#
# w^2 = c_m^2 + w_theta^2
def get_w(c_x: float, w_theta: float):
    return np.sqrt(c_x**2 + w_theta**2)


# Calculate the absolute flow angle alpha from the absolute velocity
# components.
#
# tan(alpha) = c_theta / c_m
#
# arctan2 is used so that the sign/quadrant of the velocity triangle
# is handled correctly.
def get_alpha(c_x: float, c_theta: float):
    return np.arctan2(c_theta, c_x)


# Calculate the relative flow angle beta from the relative velocity
# components.
#
# tan(beta) = w_theta / c_m
#
# beta describes the flow direction as observed in the rotating
# reference frame of the rotor.
def get_beta(c_x: float, w_theta: float):
    return np.arctan2(w_theta, c_x)


# Calculate the tangential component of absolute velocity from the
# absolute flow angle alpha.
#
# c_theta = c_m*tan(alpha)
def get_ctheta_from_alpha(c_x: float, alpha: float):
    return c_x * np.tan(alpha)


# Calculate the tangential component of absolute velocity from the
# relative flow angle beta.
#
# From the velocity triangle:
#
# c = w + U
#
# and therefore in the tangential direction:
#
# c_theta = U + w_theta
#
# with w_theta = c_m*tan(beta).
#
# This assumes beta is treated as a signed angle according to the
# velocity-triangle sign convention being used.
def get_ctheta_from_beta(U: float, c_x: float, beta: float):
    return  U + c_x * np.tan(beta)


# Calculate the tangential component of relative velocity.
#
# From:
# c = w + U
#
# therefore:
# w_theta = c_theta - U
def get_w_theta_from_c_theta(c_theta: float, U: float):
    return c_theta - U


# Calculate the tangential component of absolute velocity from the
# relative tangential velocity and blade speed.
#
# c_theta = w_theta + U
def get_c_theta_from_w_theta(w_theta: float, U: float):
    return w_theta + U


# Calculate the change in stagnation enthalpy across a rotating
# turbomachinery blade row using Euler's turbomachinery equation.
#
# delta_h0 = U2*c_theta2 - U1*c_theta1
#
# With the current sign convention:
#   delta_h0 > 0  -> energy added to fluid (compressor)
#   delta_h0 < 0  -> energy removed from fluid (turbine)
#
# Units: J/kg
def get_delta_h0(U_1: float, U_2: float, c_theta1: float, c_theta2: float):
    return U_2*c_theta2 - U_1*c_theta1


# Calculate a new stagnation temperature from a temperature and a
# specified change in stagnation enthalpy.
#
# Using:
# delta_h0 = cp*delta_T0
#
# therefore:
# T02 = T01 + delta_h0/cp
#
# For a compressor delta_h0 is positive, while for a turbine it is
# negative under the adopted sign convention.
def get_T02_from_T01(T: float, delta_h0: float, c_p: float):
    return T + delta_h0/c_p


# Convert stagnation temperature to static temperature using the
# kinetic energy of the absolute flow.
#
# h0 = h + c^2/2
#
# For a calorically perfect gas:
# h = cp*T
#
# therefore:
# T = T0 - c^2/(2*cp)
def get_T_from_T0(T0: float, c: float, cp: float):
    return T0 - c**2 / (2 * cp)


# Convert static temperature to stagnation temperature.
#
# T0 = T + c^2/(2*cp)
def get_T0_from_T(T: float, c_x: float, c_p: float):
    return T + c_x**2 / (2 * c_p)

# Calculate density using the ideal-gas equation of state.
#
# P = rho*R*T
#
# therefore:
# rho = P/(R*T)
#
# P and T should be STATIC pressure and temperature here.
def get_rho(P: float, T: float, R: float):
    return P / (R * T)


# Calculate the local speed of sound for a calorically perfect gas.
#
# a = sqrt(gamma*R*T)
#
# T should be the local STATIC temperature.
def get_speed_of_sound(T: float, gamma: float, R: float):
    return np.sqrt(gamma * R * T)


# Calculate the absolute Mach number.
#
# M = c/a
#
# where c is the magnitude of the absolute velocity and a is the
# local speed of sound based on static temperature.
def get_mach(c: float, T: float, gamma: float, R: float):
    a = get_speed_of_sound(T, gamma, R)
    return c / a


# Calculate the change in kinetic energy per unit mass associated
# with a change in absolute velocity.
#
# delta(KE) = (c2^2 - c1^2)/2
#
# Units: J/kg
#
# This is useful when applying the steady-flow energy equation to
# stationary blade rows such as stators, nozzles and diffusers.
def get_delta_h_from_velocity(c_1: float, c_2: float):
    return (c_2**2 - c_1**2)/2

def get_P2_from_P1(P_01: float, T_02: float, T_01: float, gamma: float):
    return P_01 * (T_02/T_01)**(gamma/(gamma-1))

# Convert static pressure to stagnation pressure using the
# principle of isentropic processes.
#
# dp/p = dT/T * gamma/(gamma - 1)
#
# T0/T = 1 + (gamma - 1)/2 * M^2
#
# therefore:
# P0 = P * (1 + M^2 * (gamma - 1)/2) ^ (gamma/(gamma - 1))
def get_P0_from_P(P: float, gamma: float, M: float):
    return P * (1 + M**2 * (gamma - 1)/2) ** (gamma/(gamma - 1))

# Convert stagnation pressure to static pressure.
#
# P = P0 / [1 + M^2 * (gamma - 1)/2) ^ (gamma/(gamma - 1)]
def get_P_from_P0(P0: float, gamma: float, M: float):
    return P0 / ((1 + M**2 * (gamma - 1)/2) ** (gamma/(gamma - 1)))

# Calculate the magnitude of shaft power associated with the
# stagnation enthalpy change of the fluid.
#
# |Power| = |m_dot*delta_h0|
#
# abs() means this function always returns a positive power magnitude,
# regardless of whether the machine is operating as a compressor
# (delta_h0 > 0) or turbine (delta_h0 < 0).
#
# Units: W
def get_power(delta_h0: float, m_dot: float):
    return m_dot * delta_h0

# Iteratively solve the thermodynamic and velocity state at a single
# turbomachinery station.
#
# Known quantities:
#   m_dot   = mass flow rate [kg/s]
#   A       = flow area [m^2]
#   c_theta = tangential/whirl component of absolute velocity [m/s]
#   T0      = stagnation temperature [K]
#   P0      = stagnation pressure [Pa]
#   gamma   = ratio of specific heats [-]
#   R       = specific gas constant [J/(kg K)]
#
# Other inputs:
# eps      = density convergence tolerance
# max_iter = maximum permitted number of density iterations
#
# Returns a FlowState containing the converged static, stagnation,
# and velocity properties at the station.
def solve_station(m_dot, A, c_theta, T0, P0, gamma, R, eps = 1e-12, max_iter = 1000):

    # Calculate the constant-pressure specific heat for a calorically
    # perfect gas:
    c_p = gamma * R / (gamma - 1)

    # Initial guess for density [kg/m^3].
    rho = 1

    for n in range(max_iter):

        # Calculate meridional velocity using conservation of mass:
        c_x = m_dot / (rho * A)

        # Calculate the magnitude of the absolute velocity from its
        # meridional and tangential components:
        c = np.sqrt(c_x**2 + c_theta**2)

        # Convert stagnation temperature to static temperature using
        # the steady-flow energy relationship:
        T = T0 - c**2/(2*c_p)

        # Calculate the local speed of sound using the static
        # temperature:
        a = np.sqrt(gamma * R * T)

        # Calculate the absolute Mach number:
        M = c/a

        # Calculate static pressure from stagnation pressure using the
        # isentropic perfect-gas stagnation/static pressure relation:
        P = P0 / ((1 + M**2 * (gamma - 1)/2) ** (gamma/(gamma - 1)))

        # Calculate a new density from the ideal-gas equation of state:
        new_rho = P / (R * T)

        # Check whether the density has converged.
        if abs(rho - new_rho) < eps:
            break

        rho = new_rho

    # Error message if exceed max iterations
    else:
        raise RuntimeError("Density iteration did not converge")

    # Package the converged station properties into a FlowState object.
    return FlowState(
        P=P,
        T=T,
        rho=rho,
        P0=P0,
        T0=T0,
        c_x=c_x,
        c_theta=c_theta,
        c=c,
        M=M
    )

# Solve the thermodynamic and velocity state across a single axial
# turbomachinery rotor.
#
# Known quantities:
#   m_dot   = mass flow rate [kg/s]
#   A       = flow area [m^2]
#   c_theta = tangential/whirl component of absolute velocity [m/s]
#   T0      = stagnation temperature [K]
#   P0      = stagnation pressure [Pa]
#   gamma   = ratio of specific heats [-]
#   R       = specific gas constant [J/(kg K)]
#   omega   = rotor angular velocity [rad/s]
#   r_m     = rotor mean radius [m]
#   beta    = rotor exit angle [rad]
#
# Returns a RotorResults object containing the stagnation,
# and velocity properties at the rotor exit.
def solve_axial_rotor(inlet: FlowState, blade: BladeParams, fluid: IdealGas):

    # Inlet variables
    c_z = inlet.c_x
    P_01 = inlet.P0
    T_01 = inlet.T0
    c_theta1 = inlet.c_theta

    # Blade variables
    omega = blade.N * 2 * np.pi / 60
    r_m = blade.r_m
    beta2 = blade.beta

    # Fluid variables
    gamma = fluid.gamma
    R = fluid.gamma
    c_p = fluid.c_p

    # Shaft speed
    U = get_U(r_m, omega)

    # Velocity triangles
    c_theta2 = get_ctheta_from_beta(U, c_z, beta2)

    # Euler's equation
    delta_h0 = get_delta_h0(U, U, c_theta1, c_theta2)

    # Update stagnations thermodynamic states
    T_02 = get_T02_from_T01(T_01, delta_h0, c_p)
    P_02 = get_P2_from_P1(P_01, T_02, T_01, gamma)

    # Package the results into a RotorResults object
    return RotorResults(
        c_theta=c_theta2,
        delta_h0=delta_h0,
        P0=P_02,
        T0=T_02
    )

# Solve the thermodynamic and velocity state across a single axial
# turbomachinery stator.
#
# Known quantities:
#   T0      = stagnation temperature [K]
#   P0      = stagnation pressure [Pa]
#   alpha   = stator inlet angle [rad]
#
# Returns a StatorResults object containing the stagnation,
# and velocity properties at the rotor exit.
def solve_axial_stator(inlet: FlowState, blade: BladeParams):

    # Inlet variables
    c_z = inlet.c_x
    P_01 = FlowState.P0
    T_01 = FlowState.T0

    # Blade variables
    alpha2 = blade.alpha

    # Velocity triangles
    c_theta2 = get_ctheta_from_alpha(c_z, alpha2)

    # Update thermodynamic stagnation states
    T_02 = T_01
    P_02 = P_01

    # Package the results into a StatorResults object
    return StatorResults(
        c_theta=c_theta2,
        P0=P_02,
        T0=T_02
    )

# Solve the thermodynamic and velocity state across a single axial
# turbomachinery rotor.
#
# Known quantities:
#   c_theta = tangential/whirl component of absolute velocity [m/s]
#   T0      = stagnation temperature [K]
#   P0      = stagnation pressure [Pa]
#   gamma   = ratio of specific heats [-]
#   R       = specific gas constant [J/(kg K)]
#   omega   = rotor angular velocity [rad/s]
#   r_m     = rotor mean radius [m]
#   beta    = rotor exit angle [rad]
#
# Returns a RotorResults object containing the stagnation,
# and velocity properties at the rotor exit.
def solve_radial_rotor(inlet: FlowState, blade: BladeParams, fluid: IdealGas):

    # Inlet variables
    c_m = inlet.c_x
    P_01 = inlet.P0
    T_01 = inlet.T0
    c_theta1 = inlet.c_theta

    # Blade variables
    r_1 = blade.r_1
    r_2 = blade.r_2
    alpha1 = blade.alpha
    beta2 = blade.beta
    omega = 2 * np.pi * blade.N / 60

    # Fluid variables
    gamma = fluid.gamma
    R = fluid.R
    c_p = fluid.c_p

    # Shaft speed
    U_1 = get_U(r_1, omega)
    U_2 = get_U(r_2, omega)

    # Velocity triangles
    c_theta2 = get_ctheta_from_beta(U_2, c_m, beta2)

    # Euler's equation
    delta_h0 = get_delta_h0(U_1, U_2, c_theta1, c_theta2)

    # Update thermodynamic stagnation states
    T_02 = get_T02_from_T01(T_01, delta_h0, c_p)
    P_02 = get_P2_from_P1(P_01, T_02, T_01, gamma)

    # Package the results into a RotorResults object
    return RotorResults(
        c_theta=c_theta2,
        delta_h0=delta_h0,
        P0=P_02,
        T0=T_02
    )

# Solve the thermodynamic and velocity state across a single radial
# turbomachinery nozzle/diffuser.
#
# Known quantities:
#   T0      = stagnation temperature [K]
#   P0      = stagnation pressure [Pa]
#   alpha   = stator inlet angle [rad]
#
# Returns a StatorResults object containing the stagnation,
# and velocity properties at the rotor exit.
def solve_radial_stator(inlet: FlowState, blade: BladeParams):

    # Inlet variables
    c_m = inlet.c_x
    P_01 = FlowState.P0
    T_01 = FlowState.T0

    # Blade variables
    alpha2 = blade.alpha

    # Velocity triangles
    c_theta2 = get_ctheta_from_alpha(c_m, alpha2)

    # Update thermodynamic stagnation states
    T_02 = T_01
    P_02 = P_01

    # Package the results into a StatorResults object
    return StatorResults(
        c_theta=c_theta2,
        P0=P_02,
        T0=T_02
    )