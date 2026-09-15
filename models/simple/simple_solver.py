import numpy as np

class FlowState:

    def __init__(self, P, T, rho, P0, T0, c_m, c_theta, c, w_m, w_theta, w, M, alpha, beta):

        # Thermodynamic properties
        self.P = P
        self.T = T
        self.rho = rho

        self.P0 = P0
        self.T0 = T0

        # Absolute velocity
        self.c_m = c_m
        self.c_theta = c_theta
        self.c = c

        # Relative velocity
        self.w_m = w_m
        self.w_theta = w_theta
        self.w = w

        # Flow properties
        self.M = M

        # Angles
        self.alpha = alpha
        self.beta = beta

class IdealGas:

    def __init__(self, gamma = 1.4, R = 287):

        self.gamma = gamma
        self.R = R
        self.c_p = gamma * R / (gamma - 1)