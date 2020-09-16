import numpy as np
import robodk as rdk

GLOBAL = 'global'
SILVIA = 'silvia'
GRINDER = 'grinder'
CUP = 'cup'
CROSS = 'cross'
TCP = 'tcp'
TOOL = 'tool'
PUSHERMOUNT = 'pushermount'
FILTERMOUNT = 'filtermount'
CLAMPMOUNT = 'clampmount'
FILTER = 'filter'


class Frame:

    def __init__(self, transform, label='N/A'):
        self.position = np.array([val[3] for val in transform[:3]])
        self.label = label
        self.rotation = np.array([val[:3] for val in transform[:3]])
        self.transform_np = transform
        self.transform = rdk.Mat(self.transform_np.tolist())


def read_frames(filename):
    file = open(filename, 'r')
    lines = file.readlines()
    file.close()

    frames = {}
    for line in lines:
        segments = line.rstrip().split(',')
        name = segments[0]
        values = [float(i) for i in segments[1:]]
        transform = np.array([values[0:4],
                              values[4:8],
                              values[8:12],
                              values[12:16]])
        frames[name] = Frame(transform,
                             label=name)
    return frames
