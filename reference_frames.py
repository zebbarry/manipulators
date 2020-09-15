import numpy as np
import robodk as rdk


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
        self.transform_np = np.column_stack([self.rotation, self.position])
        self.transform_np = np.vstack((self.transform_np, np.array([0, 0, 0, 1])))
        self.transform = rdk.Mat(self.transform_np.tolist())


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
