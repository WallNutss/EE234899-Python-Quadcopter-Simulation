import numpy as np
import math

class WaypointGenerator:
    def __init__(self):
        self.points = 10
        self.time = 5
        self.timestamps = np.linspace(0, self.time, self.points)
        

    def ascending_line(self, t):
        # Adjust the parameters to control the slope and ascent
        x = t
        y = 0.3 * t  # Adjust this coefficient for slower or faster ascent
        z = t

        return x,y,z
    
    def trajectory(self, state=1):
        if state == 1:
            return np.array([self.ascending_line(t) for t in self.timestamps])
