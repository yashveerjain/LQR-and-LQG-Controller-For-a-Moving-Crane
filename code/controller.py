from typing import Union
import sympy
import numpy as np
import os
import math
from sympy import Matrix, MatrixSymbol, Symbol, symbols, cos, sin, BlockMatrix
from scipy.linalg import solve_continuous_are
from scipy.signal import place_poles, step, lti
from scipy import signal


def lqr(A: np.ndarray,B: np.ndarray,Q:np.ndarray,R:np.ndarray):
    P = solve_continuous_are(A,B,Q,R)
    K_arr = -(np.linalg.inv(R)@B.T@P)
    K_arr = np.round(K_arr,3)
    K = Matrix(K_arr)
    return K

def lqg(A_tran: np.ndarray,C_tran: np.ndarray,Q:np.ndarray,R: Union[np.ndarray,int]):
    P = solve_continuous_are(A_tran,C_tran,Q,R)
    L_arr = P@C_tran@np.linalg.inv(R)
    L_arr = np.round(L_arr,3)
    L = Matrix(L_arr)
    return L