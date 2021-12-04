import numpy as np

class PathMaker:
    def __init__(self):
        pass

    def make_lines(self, x_start, y_start, x_last, y_last, N):
        x_list = np.linspace(x_start, x_last, N)
        y_list = np.linspace(y_start, y_last, N)
        line_path = np.dstack([x_list, y_list])

        return line_path