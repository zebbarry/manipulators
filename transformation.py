import numpy as np


class ReferenceFrame:

    def __init__(self, origin, label='N/A'):
        self.position = origin
        self.label = label
        self.rotation = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

    def calc_rotation(self, point):
        diff = point - self.position
        angle = np.tan(diff[1]/diff[0])

        self.rotation = np.array([[np.sin(angle), np.cos(angle), 0],
                                  [np.cos(angle), np.sin(angle), 0],
                                  [0, 0, 1]])


def main():
    filename = 'reference_frames.csv'
    file = open(filename, 'r')
    lines = file.readlines()
    file.close()

    reference_frames = {}
    for line in lines:
        name, x, y, z = line.rstrip().split(',')
        frame = ReferenceFrame(np.array([float(x), float(y), float(z)]), label=name)
        reference_frames[name] = frame

    reference_frames['coffee machine'].calc_rotation(np.array([-577.06, -446.46, 341.38]))
    print(reference_frames['coffee machine'].rotation)


if __name__ == '__main__':
    main()
