# plotting.py
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
from sklearn.metrics import mean_squared_error,mean_absolute_error
from datetime import datetime

def plot_post_simulation(time_logs, pos_logs, pos_error_logs, u_logs, sliding_surface_logs, position_ref, DEBUG, experiment):
    try:
        figsliding, axsliding = plt.subplots(3, 1)
        axsliding[0].plot(time_logs, sliding_surface_logs[:, 0], 'g',label="s_phi")
        axsliding[0].set_title("Sliding Surface of Roll Angle", fontsize=11)
        axsliding[0].legend()
        axsliding[0].grid(linestyle='--')

        axsliding[1].plot(time_logs, sliding_surface_logs[:, 1], 'r',label="s_theta")
        axsliding[1].set_title("Sliding Surface of Pitch Angle", fontsize=11)
        axsliding[1].legend()
        axsliding[1].grid(linestyle='--')

        axsliding[2].plot(time_logs, sliding_surface_logs[:, 2], 'm',label="s_psi")
        axsliding[2].set_title("Sliding Surface of Yaw Angle", fontsize=11)
        axsliding[2].legend(loc='upper right')
        axsliding[2].grid(linestyle='--')
        figsliding.subplots_adjust(wspace=0.2, hspace=0.8)
        figsliding.suptitle("Sliding Surface of Attitude Controller", fontweight="bold")
    except:
        plt.close()
        print("This is a PID Controller")
        

    figpos, axpos = plt.subplots(3, 1)
    axpos[0].plot(time_logs, pos_logs[:, 0], 'g',label="Position Data x-axis")
    axpos[0].set_ylabel('X Position(m)')
    x = np.arange(0, 50.2, 0.02)
    y = np.full_like(x, position_ref[0])
    axpos[0].plot(x,y, '--r')
    axpos[0].legend()
    axpos[0].set_title("Position of the Quadcopter", fontweight="bold")
    axpos[0].grid(linestyle='--')

    axpos[1].plot(time_logs, pos_logs[:, 1], 'r',label="Position Data y-axis")
    axpos[1].set_ylabel('Y Position(m)')
    x = np.arange(0, 50.2, 0.02)
    y = np.full_like(x, position_ref[1])
    axpos[1].plot(x,y, '--r')
    axpos[1].legend()
    axpos[1].grid(linestyle='--')

    axpos[2].plot(time_logs, pos_logs[:, 2], 'm',label="Position Data z-axis")
    axpos[2].set_xlabel('Time(s)')
    axpos[2].set_ylabel('Z Position(m)')
    x = np.arange(0, 50.2, 0.02)
    y = np.full_like(x, position_ref[2])
    axpos[2].plot(x,y, '--r')
    axpos[2].legend()
    axpos[2].grid(linestyle='--')

    figpos.align_labels()
    figpos.subplots_adjust(wspace=0.2, hspace=0.4)
    #figpos.suptitle("Position of the Quadcopter")

    figu, axu = plt.subplots(4, 1, sharex=True)
    axu[0].plot(time_logs, u_logs[:, 0], 'g', label="U1")
    axu[0].set_ylabel('U1(N)')
    axu[0].legend()
    axu[0].set_title("Control Inputs of Quadcopter", fontweight="bold")
    axu[0].grid(linestyle='--')

    axu[1].plot(time_logs, u_logs[:, 1], 'r', label="U2")
    axu[1].set_ylabel('U2(Nm)')
    axu[1].legend()
    axu[1].grid(linestyle='--')

    axu[2].plot(time_logs, u_logs[:, 2], 'm', label="U3")
    axu[2].set_ylabel('U3(Nm)')
    axu[2].legend()
    axu[2].grid(linestyle='--')

    axu[3].plot(time_logs, u_logs[:, 3], 'm', label="U4")
    axu[3].set_xlabel('Time(s)')
    axu[3].set_ylabel('U4(Nm)')
    axu[3].legend()
    axu[3].grid(linestyle='--')
    figu.subplots_adjust(wspace=0.2, hspace=0.4)
    figu.tight_layout()
    figu.align_labels()

    

    figerr, axe = plt.subplots()
    axe.plot(time_logs, pos_error_logs[:,0], '--g', label="error x-axis")
    axe.plot(time_logs, pos_error_logs[:,1], ':r', label="error y-axis")
    axe.plot(time_logs, pos_error_logs[:,2], '-.m', label="error z-axis")
    axe.grid(linestyle='--')
    axe.legend()
    axe.set_ylabel("Position Error") 
    axe.set_xlabel("Time(s)") 
    axe.set_title("Position Controller Task") 

    if DEBUG:
        pass
    else:
        try:
            figsliding.savefig(f'./data/plot/{experiment}-slidingsurface.png')
        except:
            pass
        figpos.savefig(f'./data/plot/{experiment}-positionlogs.png')    
        figu.savefig(f'./data/plot/{experiment}-controllogs.png') 
        figerr.savefig(f'./data/plot/{experiment}-errorplots.png')

    plt.show()


def reporting(Last_time, pos_logs, pos_ref, DEBUG, experiment):
        x_ref = np.full(pos_logs.shape[0], pos_ref[0])
        MSE_X = mean_squared_error(pos_logs[:,0].flatten(), x_ref.flatten())
        RMSE_X = math.sqrt(MSE_X)
        print("RMSE for Position-X: ", RMSE_X)
        #print("MAE for Position-X: ", mean_absolute_error(Quadcopter.posLogs[:,0].flatten(), x_ref.flatten()))

        y_ref = np.full(pos_logs.shape[0], pos_ref[1])
        MSE_Y = mean_squared_error(pos_logs[:,1], y_ref)
        RMSE_Y = math.sqrt(MSE_Y)
        print("RMSE for Position-Y: ", RMSE_Y)
        #print("MAE for Position-Y: ", mean_absolute_error(Quadcopter.posLogs[:,1].flatten(), y_ref.flatten()))
        
        z_ref = np.full(pos_logs.shape[0], pos_ref[2])
        MSE_Z = mean_squared_error(pos_logs[:,2], z_ref)
        RMSE_Z = math.sqrt(MSE_Z)
        print("RMSE for Position-Z: ", RMSE_Z)
        #print("MAE for Position-Z: ", mean_absolute_error(Quadcopter.posLogs[:,2].flatten(), z_ref.flatten()))
        if DEBUG:
            pass
            # print("dumping the data into one files using numpy ")
            # combine = np.column_stack((np.array(['RMSE-X','RMSE-Y','RMSE-Z']),np.array([RMSE_X,RMSE_Y,RMSE_Z])))
            # np.savetxt(f'./data/results/{experiment}-RMSE.txt', combine, fmt='%s', delimiter='=', header=f'Test taken at: {datetime.now()}\nExperiment = {experiment}\nSimulation Time = {round(Last_time,3)}s\n')

        else:
            df = pd.DataFrame(pos_logs)
            df.to_excel(f'./data/datalogs/data-{experiment}.xlsx')
            print("dumping the data into one files using numpy ")

            combine = np.column_stack((np.array(['RMSE-X','RMSE-Y','RMSE-Z']),np.array([RMSE_X,RMSE_Y,RMSE_Z])))
            np.savetxt(f'./data/results/{experiment}-RMSE.txt', combine, fmt='%s', delimiter='=', header=f'Test taken at: {datetime.now()}\nExperiment = {experiment}\nSimulation Time = {round(Last_time,3)}s\n')