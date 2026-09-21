import numpy as np
from models.simple.simple_solver import RotorResults, StatorResults, FlowState, BladeParams, IdealGas


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
    U = omega * r_m

    # Velocity triangles
    c_theta2 = U + c_z * np.tan(beta2)

    # Euler's equation
    delta_h0 = U * (c_theta2 - c_theta1)

    # Update stagnations thermodynamic states
    T_02 = T_01 + delta_h0/c_p
    P_02 = P_01 * (T_02/T_01)**(gamma/(gamma-1))

    return RotorResults(
        c_theta=c_theta2,
        delta_h0=delta_h0,
        P0=P_02,
        T0=T_02
    )

def solve_axial_stator(inlet: FlowState, blade: BladeParams):

    # Inlet variables
    c_z = inlet.c_x
    P_01 = FlowState.P0
    T_01 = FlowState.T0

    # Blade variables
    alpha2 = blade.alpha

    # Velocity triangles
    c_theta2 = c_z * np.tan(alpha2)

    # Update thermodynamic stagnation states
    T_02 = T_01
    P_02 = P_01

    return StatorResults(
        c_theta=c_theta2,
        P0=P_02,
        T0=T_02
    )

def solve_radial_rotor(inlet: FlowState, blade: BladeParams, fluid: IdealGas):

    # Inlet variables
    c_m = inlet.c_x
    P_01 = inlet.P0
    T_01 = inlet.T0

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
    U_1 = omega * r_1
    U_2 = omega * r_2

    # Velocity triangles
    c_theta1 = c_m * np.tan(alpha1)
    c_theta2 = U_2 + c_m * np.tan(beta2)

    # Euler's equation
    delta_h0 = U_2*c_theta2 - U_1*c_theta1

    # Update thermodynamic stagnation states
    T_02 = T_01 + delta_h0/c_p
    P_02 = P_01 * (T_02/T_01)**(gamma/(gamma-1))

    return RotorResults(
        c_theta=c_theta2,
        delta_h0=delta_h0,
        P0=P_02,
        T0=T_02
    )
