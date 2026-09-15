import numpy as np

class FlowState:

    def __init__(self):

        # Thermodynamic properties
        self.P = None
        self.T = None
        self.rho = None

        self.P0 = None
        self.T0 = None

        # Absolute velocity
        self.c_m = None
        self.c_theta = None
        self.c = None

        # Relative velocity
        self.w_m = None
        self.w_theta = None
        self.w = None

        # Flow properties
        self.M = None

        # Angles
        self.alpha = None
        self.beta = None

class IdealGas:

    def _init__(self, gamma = 1.4, R = 287):

        self.gamma = gamma
        self.R = R
        self.c_p = gamma * R / (gamma - 1)