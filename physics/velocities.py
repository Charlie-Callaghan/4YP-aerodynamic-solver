import numpy as np
from dataclasses import dataclass
from models.axial import OperParams, BladeParams

def get_U(r_m:float, N: float):
    return r_m * N

def get_W1(V1: float, U: float):
    return V1 - U

def get_beta1(WTheta: float, V_m: float):
    return np.arctan(WTheta/V_m)

def get_alpha1(U: float, V_m: float, beta1: float):
    return np.arctan(np.tan(beta1) - U/V_m)

