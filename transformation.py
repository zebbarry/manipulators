import numpy as np
import matplotlib.pyplot as plt
from reference_frames import *
from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox


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

    # print(frames['silvia'].transform)
    # print(frames['grinder'].transform)
    # print(frames['scraper'].transform)
    # print(frames['cup'].transform)

    silvia_frame_point = np.array([0, 218, 0, 1])
    silvia_robot_point = np.matmul(frames['silvia'].transform, silvia_frame_point)
    grinder_frame_point = np.array([157.61, 0, -250.45, 1])
    grinder_robot_point = np.matmul(frames['grinder'].transform, grinder_frame_point)
    scraper_frame_point = np.array([-80, 0, -55, 1])
    scraper_robot_point = np.matmul(frames['scraper'].transform, scraper_frame_point)

    test_points = [silvia_robot_point, grinder_robot_point, scraper_robot_point]
    plot_scene(frames, test_points)


if __name__ == '__main__':
    main()
