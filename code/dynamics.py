import sympy
import numpy as np
import math
from sympy import Matrix, symbols

# Declaring Variable
x, theta1, theta2, u = symbols(r"x \theta_1 \theta_2 u")
x_dot, theta1_dot, theta2_dot = symbols(r"\dot{x} \dot{\theta_1} \dot{\theta_2}")
x_ddot, theta1_ddot, theta2_ddot = symbols(r"\ddot{x} \ddot{\theta_1} \ddot{\theta_2}")
state = Matrix([x,x_dot, theta1, theta1_dot, theta2, theta2_dot])

# variables
m1, m2, M = symbols("m1 m2 M") # oz
l1, l2 = symbols("l1 l2") # in
g = 9.81 #m/s2

variables = {m1:100,m2:100, l1:20, l2:10, M:1000}   


N = 6 #number of tate
A = Matrix([[0,1,0,0,0,0],
            [0,0,-(m1*g)/M,0,-(m1*g)/M,0],
            [0,0,0,1,0,0],
            [0,0,-((M+m1)*g)/(M*l1),0,-(m2*g)/(M*l1),0],
            [0,0,0,0,0,1],
            [0,0,-(m1*g)/(M*l2),0,-(g*(M+m2))/(M*l2),0]])
B = Matrix([0,1/M,0,1/(M*l1),0,1/(M*l2)])

C = sympy.eye(N)



def state_space_model(A,B,C,state,input,dt):
    state_dot = A*state + B * input
    Y = C*state
    next_states = state + state_dot*dt
    return next_states,Y

def non_linear_state_space_model(x_states):
    x_ddot = (x_states[u]-(g/2)*(variables[m1]*np.sin(2*math.radians(x_states[theta1])
        )+variables[m2]*math.sin(2*math.radians(x_states[theta2]))
        )-(variables[m1]*variables[l1]*(x_states[theta1_dot]**2)*np.sin(math.radians(x_states[theta1]))
        )-(variables[m2]*variables[l2]*(x_states[theta2_dot]**2)*math.sin(math.radians(x_states[theta2])))
         )/(variables[M]+variables[m1]*((np.sin(math.radians(x_states[theta1])))**2)+variables[m2]*((np.sin(math.radians(x_states[theta2])))**2))
    theta1_ddot = x_ddot*np.cos(math.radians(x_states[theta1]))/variables[l1] - g*np.sin(math.radians(x_states[theta1]))/variables[l1]
    theta2_ddot = x_ddot*np.cos(math.radians(x_states[theta2]))/variables[l2] - g*np.sin(math.radians(x_states[theta2]))/variables[l2]
    non_linear_state = Matrix([x_states[x_dot],x_ddot, x_states[theta1_dot], theta1_ddot, x_states[theta2_dot], theta2_ddot])
    return non_linear_state