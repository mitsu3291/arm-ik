import numpy as np
import math

class PathMaker:
    def __init__(self):
        pass

    def make_lines(self, x_start, y_start, x_last, y_last, N):
        x_list = np.linspace(x_start, x_last, N)
        y_list = np.linspace(y_start, y_last, N)
        line_path = np.dstack([x_list, y_list])

        return line_path

    def make_circle(self, center_x, center_y, radius, th_start, th_last, N):
        th_list = np.linspace(th_start, th_last, N)
        x_list = [center_x + radius*math.cos(theta) for theta in th_list]
        y_list = [center_y + radius*math.sin(theta) for theta in th_list]
        circle_path = np.dstack([x_list, y_list])

        return circle_path

if __name__ == "__main__":
    path_maker = PathMaker()
    path_maker.make_circle(0,0,1,-math.pi/2, math.pi/2, 3)