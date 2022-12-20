from typing import Union
import sympy
import numpy as np
import os
import math
from sympy import Matrix, MatrixSymbol, Symbol, symbols, cos, sin, BlockMatrix
from scipy.linalg import solve_continuous_are
from scipy.signal import place_poles, step, lti
from scipy import signal
import code.dynamics as dy

class Controller:
    def lqr(self, A: np.ndarray,B: np.ndarray,Q:np.ndarray,R:np.ndarray):
        P = solve_continuous_are(A,B,Q,R)
        K_arr = -(np.linalg.inv(R)@B.T@P)
        self.K_arr = np.round(K_arr,3)
        K = Matrix(K_arr)
        return K

    def lqg(self, A_tran: np.ndarray,C_tran: np.ndarray,Q:np.ndarray,R: Union[np.ndarray,int]):
        P = solve_continuous_are(A_tran,C_tran,Q,R)
        L_arr = P@C_tran@np.linalg.inv(R)
        self.L_arr = np.round(L_arr,3)
        L = Matrix(L_arr)
        return L

    def ode(self, t,Y):
        u_temp = (self.K_arr@Y)[0]
        X_states = {dy.x:Y[0],dy.x_dot:Y[1], dy.theta1:Y[2], dy.theta1_dot:Y[3], dy.theta2:Y[4], dy.theta2_dot:Y[5], dy.u:u_temp}
        Y_sym = dy.non_linear_state_space_model(X_states)
        Y_arr = np.array(Y_sym,dtype=np.float32).squeeze()
        return Y_arr
    
    def ode_for_observer(self, A, need_K=True):
        def ode(t,Y):
            if need_K:
                u_temp = (self.K_arr@Y[:6])[0]
            else:
                u_temp = 0
            X_states = {dy.x:Y[0],dy.x_dot:Y[1], dy.theta1:Y[2], dy.theta1_dot:Y[3], dy.theta2:Y[4], dy.theta2_dot:Y[5], dy.u:u_temp}
            Y_sym = dy.non_linear_state_space_model(X_states)
            et = (A@Y).squeeze()
            Y_arr = np.array(Y_sym,dtype=np.float32).squeeze()
            Y_arr = np.hstack([Y_arr,et[6:]])
            return Y_arr
        return ode