import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Plotter:
    def __init__(self, j1x_list, j1y_list, j2x_list, j2y_list, ref_path):
        self.j1x_list = j1x_list
        self.j1y_list = j1y_list
        self.j2x_list = j2x_list
        self.j2y_list = j2y_list
        self.ref_path = ref_path

    def animation(self):
        # 図を用意
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect="equal")

        x_ref_list = []
        y_ref_list = []
        for j in range(len(self.ref_path[0])):
            x_ref_list.append(self.ref_path[0][j][0])
            y_ref_list.append(self.ref_path[0][j][1])

        def update(i):
            ax.cla() # ax をクリア
            ax.set_xlim(-0.5,2.0)
            ax.set_ylim(-1.5,1.5)
            ax.plot([0, self.j1x_list[i], self.j2x_list[i]], [0, self.j1y_list[i], self.j2y_list[i]], color="m")
            ax.scatter(0,0, color="m")
            ax.scatter(self.j1x_list[i], self.j1y_list[i], color="m")
            ax.scatter(self.j2x_list[i], self.j2y_list[i], color="m")

            ax.plot(x_ref_list, y_ref_list)

        anim = FuncAnimation(fig, update, frames=range(len(self.j1x_list)), interval=10)
        anim.save("2DOF_analitical_ik.gif", writer='Pillow')
        plt.show()