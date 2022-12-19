import matplotlib.pyplot as plt
import os


def plot_respone(output_arr, time_steps, N, state, plot_title = "Controller Response", Plot_filename = "Controller Response", image_dir="response_images", save = False):
    os.makedirs(image_dir,exist_ok=True)
    try:
        N_min = min([output_arr.shape[1],N])
    except:
        N_min = 1
    print(N_min)
    fig, axs = plt.subplots(N_min,1,sharex=True)
    fig.suptitle(plot_title)
    fig.tight_layout()
    if N_min>1:
        # fig.a ("time steps (sec)")
        for n in range(N_min):
            axs[n].set_ylabel(rf"${state[n]}$")
            axs[n].set_xlabel("Time steps (sec)")
            axs[n].plot(time_steps, output_arr[:,n])
            axs[n].label_outer()
    else:
        axs.set_ylabel(rf"${state}$")
        axs.set_xlabel("Time steps (sec)")
        axs.plot(time_steps, output_arr[:])
        axs.label_outer()
    # plt.savefig("InitialResponseWithoutController.png")
    if save:
        plt.savefig(f"{image_dir}/{'_'.join(Plot_filename.split())}.png", bbox_inches='tight')