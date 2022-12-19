
import math
import sympy 
from sympy import Matrix, MatrixSymbol, Symbol, symbols, cos, sin, BlockMatrix
import numpy as np
from tqdm import tqdm
import code.dynamics as dy

def state_space(A,B,C,K,X_states,state,ue_controller=True):
    total_time = 500
    dt = .01

    Ts = np.arange(0,total_time,dt)

    curr_state = state.subs(X_states)
    output = []
    # print("\ncurrent_states: ",X_states)
    if ue_controller:
        optimal_control_input = K
        A_new = A + B*optimal_control_input
    else:
        # Take initial condition here input u i 0.
        optimal_control_input = 0
        A_new = A

    for i in tqdm(Ts):
        u_temp = (optimal_control_input*curr_state)[0]
        X_states = {dy.x:curr_state[0],dy.x_dot:curr_state[1], dy.theta1:curr_state[2], dy.theta1_dot:curr_state[3], dy.theta2: curr_state[4], dy.theta2_dot:curr_state[5], dy.u:u_temp}
        next_state,Y = dy.state_space_model(A_new,B,C,curr_state,0,dt)
        
        curr_state=next_state

        output.append(Y)
        
    output_arr = np.array(output,dtype=np.float32).squeeze()

    return output_arr, Ts

def state_space_observer(A,B,C,L,K,X_states,states):
    total_time = 500
    dt = .01

    Ts = np.arange(0,total_time,dt)
    curr_state = states.subs(X_states)
    output = []
    
    A_block = Matrix(BlockMatrix([[A+B*K,-B*K],[sympy.zeros(*A.shape),A-L*C]]))
    B_block = Matrix(BlockMatrix([[B],[sympy.zeros(*B.shape)]]))
    C_block = Matrix(BlockMatrix([C,sympy.zeros(*C.shape)]))
    D_block = sympy.zeros(C.shape[0],1)
    optimal_control_input = Matrix(BlockMatrix([K,sympy.zeros(*K.shape)]))
    
    A_block_arr = np.array(A_block,dtype=np.float32)
    B_block_arr = np.array(B_block,dtype=np.float32)
    C_block_arr = np.array(C_block,dtype=np.float32)
    D_block_arr = np.array(D_block,dtype=np.float32)
    inp = 0
    non_linear_outputs = []
    all_states = []
    for i in tqdm(Ts):
        u_temp = (optimal_control_input*curr_state)[0]
        X_states = {dy.x:curr_state[0],dy.x_dot:curr_state[1], dy.theta1:curr_state[2], dy.theta1_dot:curr_state[3], dy.theta2: curr_state[4], dy.theta2_dot:curr_state[5], dy.u:u_temp}
        all_states.append(curr_state)
        
        next_state,Y = dy.state_space_model(A_block,B_block,C_block,curr_state,0,dt)    
        curr_state=next_state
        # print(curr_state)
        output.append(Y)
        # print(non_linear_output)
    output_arr = np.array(output,dtype=np.float32).squeeze()
    # print(non_linear_outputs)
    output_arr = np.array(output,dtype=np.float32).squeeze()
    all_states_arr = np.array(all_states,dtype=np.float32)
    return output_arr, all_states_arr, Ts, (A_block_arr,B_block_arr,C_block_arr,D_block_arr)