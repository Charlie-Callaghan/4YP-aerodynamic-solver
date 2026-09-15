import numpy as np
from models.simple.simple_solver import FlowState

def thermo_iter(m_dot, A, c_theta, T0, P0, gamma, R, eps = 1e-12, max_iter = 1000):

    rho = 1

    for n in range(max_iter):

        c_m = m_dot / (rho * A)

        c = np.sqrt(c_m**2 + c_theta**2)

        T = T0 - c**2/(2*c_p)

        a = np.sqrt(gamma * R * T)

        M = c/a

        P = P0 / ((1 + M**2 * (gamma - 1)/2) ** (gamma/(gamma - 1)))

        new_rho = P / (R * T)

        if abs(rho - new_rho) < eps:
            break

        rho = new_rho
    else:
        raise RuntimeError("Density iteration did not converge")

    return FlowState(
        P=P,
        T=T,
        rho=rho,
        P0=P0,
        T0=T0,
        c_m=c_m,
        c_theta=c_theta,
        c=c,
        M=M
    )