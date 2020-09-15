import numpy as np
import matplotlib.pyplot as plt
from reference_frames import *
from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import matplotlib as plt


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
