import numpy as np
import robodk as rdk

ROBOT = 'robot'
SILVIA2ROBOT = 'silvia2robot'
GRINDER2ROBOT = 'grinder2robot'
CUP2ROBOT = 'cup2robot'
TOOLS2ROBOT = 'tools2robot'
SCRAPER2ROBOT = 'scraper2robot'
TOOL2TCP = 'tool2tcp'


class ReferenceFrame:

    def __init__(self, transform, label='N/A'):
        self.position = np.array([val[3] for val in transform[:3]])
        self.label = label
        self.rotation = np.array([val[:3] for val in transform[:3]])
        self.transform_np = transform
        self.transform = rdk.Mat(self.transform_np.tolist())

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
        self.transform_np = np.column_stack([self.rotation, self.position])
        self.transform_np = np.vstack((self.transform, np.array([0, 0, 0, 1])))
        self.transform = rdk.Mat(self.transform_np.tolist())


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
                                               [np.sin(angle), np.cos(angle), 0],
                                               [0, 0, 1]])

    frames[CUP2ROBOT].rotation = np.array([[-1, 0, 0],
                                           [0, -1, 0],
                                           [0, 0, 1]])

    angle = 50
    frames[TOOL2TCP].rotation = np.array([[1, 0, 0],
                                          [0, 1, 0],
                                          [0, 0, -1]])
    for name, frame in frames.items():
        frame.calc_transform()
    return frames
