import numpy as np
import robodk as rdk

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
        self.transform_np = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
        self.transform = rdk.Mat([[1, 0, 0, 0],
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
