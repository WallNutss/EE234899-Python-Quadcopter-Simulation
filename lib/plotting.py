# plotting.py
import matplotlib.pyplot as plt

def plot_post_simulation(time_logs, pos_logs, u_logs, sliding_surface_logs):
    try:
        figsliding, axsliding = plt.subplots(3, 1)
        axsliding[0].plot(time_logs, sliding_surface_logs[:, 0], 'g')
        axsliding[0].set_title("Sliding Surface of Roll Angle", fontsize=11)
        axsliding[0].grid(linestyle='--')

        axsliding[1].plot(time_logs, sliding_surface_logs[:, 1], 'r')
        axsliding[1].set_title("Sliding Surface of Pitch Angle", fontsize=11)
        axsliding[1].grid(linestyle='--')

        axsliding[2].plot(time_logs, sliding_surface_logs[:, 2], 'm')
        axsliding[2].set_title("Sliding Surface of Yaw Angle", fontsize=11)
        axsliding[2].grid(linestyle='--')
        figsliding.tight_layout()
        figsliding.subplots_adjust(top=0.88)
        figsliding.suptitle("Sliding Surface of Attitude Controller")
    except:
        plt.close()
        print("This is a PID Controller")
        

    figpos, axpos = plt.subplots(3, 1)
    axpos[0].plot(time_logs, pos_logs[:, 0], 'g')
    axpos[0].set_ylabel('X Position(m)')
    axpos[0].grid(linestyle='--')

    axpos[1].plot(time_logs, pos_logs[:, 1], 'r')
    axpos[1].set_ylabel('Y Position(m)')
    axpos[1].grid(linestyle='--')

    axpos[2].plot(time_logs, pos_logs[:, 2], 'm')
    axpos[2].set_xlabel('Time(s)')
    axpos[2].set_ylabel('Z Position(m)')
    axpos[2].grid(linestyle='--')
    figpos.tight_layout()
    figpos.subplots_adjust(top=0.91)
    figpos.align_labels()
    figpos.suptitle("Position of the Quadcopter")

    figu, axu = plt.subplots(4, 1, sharex=True)
    axu[0].plot(time_logs, u_logs[:, 0], 'g')
    axu[0].set_ylabel('U1(N)')
    axu[0].grid(linestyle='--')

    axu[1].plot(time_logs, u_logs[:, 1], 'r')
    axu[1].set_ylabel('U2(Nm)')
    axu[1].grid(linestyle='--')

    axu[2].plot(time_logs, u_logs[:, 2], 'm')
    axu[2].set_ylabel('U3(Nm)')
    axu[2].grid(linestyle='--')

    axu[3].plot(time_logs, u_logs[:, 3], 'm')
    axu[3].set_xlabel('Time(s)')
    axu[3].set_ylabel('U4(Nm)')
    axu[3].grid(linestyle='--')

    figu.suptitle("Control Input of Quadcopter")
    figu.align_labels()
    plt.show()
