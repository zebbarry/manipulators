import numpy as np
import matplotlib.pyplot as plt


class ReferenceFrame:

    def __init__(self, origin, label='N/A', ref_point=None):
        self.position = origin
        self.label = label
        self.ref_point = ref_point
        self.rotation = np.array([[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 1]])
        self.transform = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])

    def calc_angle(self):
        diff = self.ref_point - self.position
        angle = np.arctan(diff[1] / diff[0])
        return angle

    def vectors(self):
        a = np.matmul(self.rotation, self.position)
        x_axis = [self.position[0], a[0]]
        y_axis = [self.position[1], a[1]]
        return x_axis, y_axis

    def calc_transform(self):
        self.transform = np.column_stack([self.rotation, self.position])
        self.transform = np.vstack((self.transform, np.array([0, 0, 0, 1])))


def plot_scene(frames, test_points):
    silvia_robot_point, grinder_robot_point, scraper_robot_point = test_points

    #Robot
    plt.figure()
    plt.ylabel('Y (mm)')
    plt.xlabel('X (mm)')
    plt.axis('equal')
    plt.plot([frames['robot'].position[0], frames['robot'].position[0]+50],
             [frames['robot'].position[1], frames['robot'].position[1]],
             '-r')
    plt.plot([frames['robot'].position[0], frames['robot'].position[0]],
             [frames['robot'].position[1], frames['robot'].position[1]+50],
             '-g')

    scale = 50
    for name, frame in frames.items():
        if name != 'robot':
            rotation = frame.rotation
            plt.plot([frame.position[0], frame.position[0] + scale * rotation[0][0]],
                     [frame.position[1], frame.position[1] + scale * rotation[1][0]],
                     '-r')
            plt.plot([frame.position[0], frame.position[0] + scale * rotation[0][1]],
                     [frame.position[1], frame.position[1] + scale * rotation[1][1]],
                     '-g')

    #Silvia
    plt.plot([frames['silvia'].ref_point[0]],
             [frames['silvia'].ref_point[1]],
             'ob')
    plt.plot([silvia_robot_point[0]],
             [silvia_robot_point[1]],
             '+', color='orange')

    #Grinder
    plt.plot([frames['grinder'].ref_point[0]],
             [frames['grinder'].ref_point[1]],
             'ob')
    plt.plot([grinder_robot_point[0]],
             [grinder_robot_point[1]],
             '+', color='orange')

    #Scraper
    plt.plot([frames['scraper'].ref_point[0]],
             [frames['scraper'].ref_point[1]],
             'ob')
    plt.plot([scraper_robot_point[0]],
             [scraper_robot_point[1]],
             '+', color='orange')


def calc_reference_frames(frames):
    angle = frames['silvia'].calc_angle()
    frames['silvia'].rotation = np.array([[-np.sin(angle), -np.cos(angle), 0],
                                          [np.cos(angle), -np.sin(angle), 0],
                                          [0, 0, 1]])

    angle = frames['grinder'].calc_angle()
    frames['grinder'].rotation = np.array([[-np.cos(angle), np.sin(angle), 0],
                                          [-np.sin(angle), -np.cos(angle), 0],
                                          [0, 0, 1]])

    angle = frames['scraper'].calc_angle()
    frames['scraper'].rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                                          [np.sin(angle), -np.cos(angle), 0],
                                          [0, 0, 1]])

    frames['cup'].rotation = np.array([[-1, 0, 0],
                                       [0, -1, 0],
                                       [0, 0, 1]])
    for name, frame in frames.items():
        frame.calc_transform()
    return frames


def main():
    filename = 'reference_frames.csv'
    file = open(filename, 'r')
    lines = file.readlines()
    file.close()

    frames = {}
    for line in lines:
        segments = line.rstrip().split(',')
        if len(segments) > 4:
            name, x, y, z, a, b, c = segments
            frames[name] = ReferenceFrame(np.array([float(x), float(y), float(z)]),
                                          label=name, ref_point=np.array([float(a), float(b), float(c)]))
        else:
            name, x, y, z = segments
            frames[name] = ReferenceFrame(np.array([float(x), float(y), float(z)]),
                                          label=name)

    frames = calc_reference_frames(frames)

    print(frames['silvia'].transform)
    print(frames['grinder'].transform)
    print(frames['scraper'].transform)
    print(frames['cup'].transform)

    silvia_frame_point = np.array([0, 218, 0, 1])
    silvia_robot_point = np.matmul(frames['silvia'].transform, silvia_frame_point)
    grinder_frame_point = np.array([157.61, 0, -250.45, 1])
    grinder_robot_point = np.matmul(frames['grinder'].transform, grinder_frame_point)
    scraper_frame_point = np.array([-80, 0, -55, 1])
    scraper_robot_point = np.matmul(frames['scraper'].transform, scraper_frame_point)

    test_points = [silvia_robot_point, grinder_robot_point, scraper_robot_point]
    plot_scene(frames, test_points)
    plt.show()


if __name__ == '__main__':
    main()
