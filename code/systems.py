
import numpy as np
from tqdm import tqdm
import code.dynamics as dy

def state_space(A,B,C,K,X_states,state,ue_controller=True):
    total_time = 500
    dt = .01

    Ts = np.arange(0,total_time,dt)

    curr_state = state.subs(X_states)
    output = []
    non_linear_outputs = []
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
        non_linear_output = dy.non_linear_state_space_model(X_states)
        curr_state=next_state
        # print(curr_state)
        # print(X_states)
        # print("#######")
        output.append(Y)
        non_linear_outputs.append(non_linear_output)
        # print(non_linear_output)
    output_arr = np.array(output,dtype=np.float32).squeeze()
    # print(non_linear_outputs)
    non_linear_output_arr = np.array(non_linear_outputs,dtype=np.float32).squeeze()

    return output_arr, non_linear_output_arr, Ts