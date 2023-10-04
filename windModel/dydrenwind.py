import numpy as np
import math
from scipy import signal
import matplotlib.pyplot as plt

class DydrenWind:
    def __init__(self):
        # Time Information
        self.Time = 100
        self.dt   = 0.02
        self.timesamples = np.arange(0, self.Time + self.dt, self.dt)

        # Information basic for Dydren Wind Disturbance Model
        self.height         = 6.076         # m  ,  the height of the aircraft
        self.airspeed       = 2             # m/s,  the speed wind disturbance we want, make sure it's not over the limit what the drone can do
        self.turbulancelvl  = 15 * 0.514444 # Knots to m/s
        
        # Generate white gaussian noise 
        self.mean            = 0
        self.std             = 1

    def u_transfer_function(self):
        length_u = self.height / ((0.177 + 0.000823*self.height)**(1.2))
        sigma_w = 0.1 * self.turbulancelvl 
        sigma_u = sigma_w / ((0.177 + 0.000823*self.height) ** (0.4))
        num_u = [sigma_u * (math.sqrt((2 * length_u) / (math.pi * self.airspeed))) * self.airspeed]
        den_u = [length_u, self.airspeed]
        H_u = signal.TransferFunction(num_u, den_u)
        return H_u
    
    def v_transfer_function(self):
        length_v = self.height / ((0.177 + 0.000823 * self.height)**(1.2))
        sigma_w = 0.1 * self.turbulancelvl
        sigma_v = sigma_w / ((0.177 + 0.000823 * self.height) ** (0.4))
        b = sigma_v * (math.sqrt((length_v) / (math.pi * self.airspeed)))
        Lv_V = length_v / self.airspeed
        num_v = [(math.sqrt(3)*Lv_V*b), b]
        den_v = [(Lv_V**2), 2*Lv_V, 1]
        H_v = signal.TransferFunction(num_v, den_v)
        return H_v
    def w_transfer_function(self):
        length_w = self.height
        sigma_w = 0.1 * self.turbulancelvl
        c = sigma_w * (math.sqrt((length_w) / (math.pi * self.airspeed)))
        Lw_V = length_w / self.airspeed
        num_w = [(math.sqrt(3)*Lw_V*c), c]
        den_w = [(Lw_V**2), 2*Lw_V, 1]
        H_v = signal.TransferFunction(num_w, den_w)
        return H_v
    
    def Model(self):
        # the random number seed used same as from SIMULINK blockset
        np.random.seed(23341)
        samples1 = 10*np.random.normal(self.mean, self.std, size= self.timesamples.shape[0])

        np.random.seed(23342)
        samples2 = 10*np.random.normal(self.mean, self.std, size= self.timesamples.shape[0])

        np.random.seed(23343)
        samples3 = 10*np.random.normal(self.mean, self.std, size= self.timesamples.shape[0])

        #generate tranfer function for dryden wind speeds in along wind direction, cross-wind, and vertical-wind directions
        tf_u = self.u_transfer_function()
        tf_v = self.v_transfer_function()
        tf_w = self.w_transfer_function()

        # compute response to tranfer function
        tout1, y1, x1 = signal.lsim(tf_u, samples1, self.timesamples)
        # tout1, y1, x1 = signal.lsim(tf_u, n1, t_w)
        # covert obtained values to meters/second
        y1_f = [i * 0.305 for i in y1]
        
        tout2, y2, x2 = signal.lsim(tf_v, samples2, self.timesamples)
        # tout2, y2, x2 = signal.lsim(tf_v, n2, t_w)
        y2_f = [i * 0.305 for i in y2]
        tout3, y3, x3 = signal.lsim(tf_w, samples3, self.timesamples)
        # tout3, y3, x3 = signal.lsim(tf_w, n3, t_w)
        y3_f = [i * 0.305 for i in y3]

        return np.hstack((np.array(y1_f).reshape(5001,1), np.array(y2_f).reshape(5001,1), np.array(y3_f).reshape(5001,1)))



# dydren = DydrenWind()
# wind = dydren.Model()
# # print(wind[1,:])
# #ans = np.hstack((y1_f,y2_f,y3_f))
# y1_f = wind[:,0].flatten()
# y2_f = wind[:,1].flatten()
# y3_f = wind[:,2].flatten()
# #print(ans)

# #plots for along-wind velocities generated using Python
# plt.figure(1)

# # t_p = time(s)
# plt.plot(dydren.timesamples, y1_f, 'b')
# # plt.plot(t_w, y1_f, 'b')
# plt.ylabel('along-wind in m/s (P)')
# plt.xlabel('time in seconds')
# plt.grid(True)

# #plots for cross-wind velocities generated using Python
# plt.figure(2)

# plt.plot(dydren.timesamples, y2_f, 'r')
# # plt.plot(t_w, y2_f, 'r')
# plt.ylabel('cross-wind in m/s (P)')
# plt.xlabel('time in seconds')
# plt.grid(True)

# #plots for vertical-wind velocities generated using Python
# plt.figure(3)

# plt.plot(dydren.timesamples, y3_f, 'g')
# # plt.plot(t_w, y3_f, 'g')
# plt.ylabel('vertical-wind in m/s (P)')
# plt.xlabel('time in seconds')
# plt.grid(True)

# # Show all plots
# plt.show()
