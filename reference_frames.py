import numpy as np
import matplotlib.pyplot as plt

ROBOT = 'robot'
SILVIA2ROBOT = 'SILVIA2ROBOT'
GRINDER2ROBOT = 'GRINDER2ROBOT'
CUP2ROBOT = 'CUP2ROBOT'
TOOLS2ROBOT = 'TOOLS2ROBOT'
SCRAPER2ROBOT = 'SCRAPER2ROBOT'
TOOL2TCP = 'TOOL2TCP'


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

    # Robot
    plt.figure()
    plt.ylabel('Y (mm)')
    plt.xlabel('X (mm)')
    plt.axis('equal')
    plt.plot([frames[ROBOT].position[0], frames[ROBOT].position[0] + 50],
             [frames[ROBOT].position[1], frames[ROBOT].position[1]],
             '-r')
    plt.plot([frames[ROBOT].position[0], frames[ROBOT].position[0]],
             [frames[ROBOT].position[1], frames[ROBOT].position[1] + 50],
             '-g')

    scale = 50
    for name, frame in frames.items():
        if name != ROBOT:
            rotation = frame.rotation
            plt.plot([frame.position[0], frame.position[0] + scale * rotation[0][0]],
                     [frame.position[1], frame.position[1] + scale * rotation[1][0]],
                     '-r')
            plt.plot([frame.position[0], frame.position[0] + scale * rotation[0][1]],
                     [frame.position[1], frame.position[1] + scale * rotation[1][1]],
                     '-g')

    # Silvia
    plt.plot([frames[SILVIA2ROBOT].ref_point[0]],
             [frames[SILVIA2ROBOT].ref_point[1]],
             'ob')
    plt.plot([silvia_robot_point[0]],
             [silvia_robot_point[1]],
             '+', color='orange')

    # Grinder
    plt.plot([frames[GRINDER2ROBOT].ref_point[0]],
             [frames[GRINDER2ROBOT].ref_point[1]],
             'ob')
    plt.plot([grinder_robot_point[0]],
             [grinder_robot_point[1]],
             '+', color='orange')

    # Scraper
    plt.plot([frames[SCRAPER2ROBOT].ref_point[0]],
             [frames[SCRAPER2ROBOT].ref_point[1]],
             'ob')
    plt.plot([scraper_robot_point[0]],
             [scraper_robot_point[1]],
             '+', color='orange')
    plt.show()


def calc_reference_frames(frames):
    angle = frames[SILVIA2ROBOT].calc_angle()
    frames[SILVIA2ROBOT].rotation = np.array([[-np.sin(angle), -np.cos(angle), 0],
                                          [np.cos(angle), -np.sin(angle), 0],
                                          [0, 0, 1]])

    angle = frames[GRINDER2ROBOT].calc_angle()
    frames[GRINDER2ROBOT].rotation = np.array([[-np.cos(angle), np.sin(angle), 0],
                                           [-np.sin(angle), -np.cos(angle), 0],
                                           [0, 0, 1]])

    angle = frames[SCRAPER2ROBOT].calc_angle()
    frames[SCRAPER2ROBOT].rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                                           [np.sin(angle), -np.cos(angle), 0],
                                           [0, 0, 1]])

    frames[CUP2ROBOT].rotation = np.array([[-1, 0, 0],
                                           [0, -1, 0],
                                           [0, 0, 1]])

    angle = 50
    frames[TOOL2TCP].rotation = np.array([[0, 0, 0],
                                          [0, 0, 0],
                                          [0, 0, -1]])
    for name, frame in frames.items():
        frame.calc_transform()
    return frames
