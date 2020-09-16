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
