import robodk as rdk

GLOBAL = 'global'
SILVIA = 'silvia'
GRINDER = 'grinder'
CUPSTACK = 'cupstack'
CUP = 'cup'
CROSS = 'cross'
TCP = 'tcp'
TOOL = 'tool'
GRINDERMOUNT = 'grindermount'
FILTERMOUNT = 'filtermount'
CUPMOUNT = 'cupmount'
FILTER = 'filter'
BALL = 'ball'
SILVIAPOWERBUTTON = 'silviapowerbutton'
SILVIAPOWERBUTTONPUSH = 'silviapowerbuttonpush'


def read_frames(filename):
    file = open(filename, 'r')
    lines = file.readlines()
    file.close()

    frames = {}
    for line in lines:
        segments = line.rstrip().split(',')
        name = segments[0]
        values = [float(i) for i in segments[1:]]
        transform = rdk.Mat([values[0:4],
                             values[4:8],
                             values[8:12],
                             values[12:16]])
        frames[name] = transform
    return frames


def read_joint_angles(filename):
    file = open(filename, 'r')
    lines = file.readlines()
    file.close()

    joint_angles = {}
    for line in lines:
        segments = line.rstrip().split(',')
        name = segments[0]
        values = [float(i) for i in segments[1:]]
        angles = rdk.Mat(values)
        joint_angles[name] = angles
    return joint_angles
