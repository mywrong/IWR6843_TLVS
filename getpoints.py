
from detected_points import Detected_Points
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import juggle_axes

FLOOR = -10
CEILING = 10

class AnimatedScatter(object):
    def __init__(self, numpoints=5):
        detected_points=Detected_Points()
        self.stream = detected_points.data_stream_iterator('COM4','COM3',1000)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111,projection = '3d')
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=10, 
                                           init_func=self.setup_plot, blit=True)

    def setup_plot(self):
        data=next(self.stream)
        x, y, z = np.transpose(data)

        c = ['b', 'r', 'g', 'y', 'm']
        self.scat = self.ax.scatter(x, y, z,c=c, s=20, animated=True)
        self.ax.set_xlim3d(FLOOR, CEILING)
        self.ax.set_ylim3d(FLOOR, CEILING)
        self.ax.set_zlim3d(FLOOR, CEILING)

        return self.scat,

    def update(self, i):
        data=next(self.stream)
        x, y, z = np.transpose(data)

        self.scat._offsets3d = juggle_axes(x, y, z, 'z')
        plt.draw()
   
        return self.scat,

    def show(self):
        plt.show()

if __name__ == "__main__":
    a = AnimatedScatter()
    a.show()


