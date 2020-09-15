import numpy as np
import matplotlib.pyplot as plt


class ReferenceFrame:

    def __init__(self, origin, label='N/A'):
        self.position = origin
        self.label = label
        self.rotation = np.array([[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 1]])

    def calc_rotation(self, point):
        diff = point - self.position
        angle = np.arctan(diff[1] / diff[0])
        print(np.degrees(angle))

        self.rotation = np.array([[np.sin(angle), np.cos(angle), 0],
                                  [np.cos(angle), np.sin(angle), 0],
                                  [0, 0, 1]])

    def vectors(self):
        a = np.matmul(self.rotation, self.position)
        x_axis = [self.position[0], a[0]]
        y_axis = [self.position[1], a[1]]
        return x_axis, y_axis


def main():
    filename = 'reference_frames.csv'
    file = open(filename, 'r')
    lines = file.readlines()
    file.close()

    frames = {}
    for line in lines:
        name, x, y, z = line.rstrip().split(',')
        frames[name] = ReferenceFrame(np.array([float(x), float(y), float(z)]), label=name)
    machine_point = np.array([-577.06, -446.46, 341.38])
    frames['coffee machine'].calc_rotation(machine_point)
    print(frames['coffee machine'].rotation)

    plt.figure()
    x_axis, y_axis = frames['robot'].vectors()
    plt.plot([frames['robot'].position[0], frames['robot'].position[0]+50],
             [frames['robot'].position[1], frames['robot'].position[1]],
             '-r')
    plt.plot([frames['robot'].position[0], frames['robot'].position[0]],
             [frames['robot'].position[1], frames['robot'].position[1]+50],
             '-g')

    x_axis, y_axis = frames['coffee machine'].vectors()
    rotation = frames['coffee machine'].rotation
    plt.plot([frames['coffee machine'].position[0], frames['coffee machine'].position[0] - 50*rotation[0][0]],
             [frames['coffee machine'].position[1], frames['coffee machine'].position[1] + 50*rotation[1][0]],
             '-r')
    plt.plot([frames['coffee machine'].position[0], frames['coffee machine'].position[0] - 50*rotation[0][1]],
             [frames['coffee machine'].position[1], frames['coffee machine'].position[1] - 50*rotation[1][1]],
             '--g')
    # plt.plot([frames['coffee machine'].position[0], machine_point[0]],
    #          [frames['coffee machine'].position[1], machine_point[1]],
    #          '-k')
    plt.show()


if __name__ == '__main__':
    main()
