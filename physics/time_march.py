import numpy as np

rho = 1
A = 1
r_1 = 2
r_2 = 1
alpha1 = 0
beta2 = np.pi/4
m_dot = 1
I = 1
tau_load = 1

delta_t = 0.1
max_iter = 1000
eps = 1e-12

def solve_turbine(m_dot, A, r_1, r_2, alpha1, beta2, rho, I, tau_load, eps = 1e-12, max_iter = 1000):

    omega = 1

    for n in range(max_iter):

        U_1 = get_U(omega, r_1)
        U_2 = get_U(omega, r_2)

        c_m = get_c_m(m_dot, rho, A)

        c_theta1 = get_ctheta_from_alpha(c_x, alpha1)
        c_theta2 = get_ctheta_from_beta(U, c_x, beta2)

        delta_h0 = get_delta_h0(U_1, U_2, c_theta1, c_theta2)

        W_s_dot = get_power(delta_h0, m_dot)

        domega_dt = (- m_dot/omega * delta_h0 - tau_load)/I

        omega_new = omega + delta_t * domega_dt

        omega = omega_new

    return FlowState(
            rho=rho,
            c_m=c_m,
            c_theta=c_theta,
            c=c
        )

