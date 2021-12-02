from PathMaker import PathMaker

if __name__ == "__main__":
    path_maker = PathMaker()
    line_path = path_maker.make_lines(x_start=0, y_start=0, x_last=1, y_last=1, N=3)
    print(line_path)