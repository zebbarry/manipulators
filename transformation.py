import matplotlib.pyplot as plt
from reference_frames import *
import numpy as np


def plot_scene(frames, test_points):
    silvia_robot_point, grinder_robot_point, cross_robot_point = test_points

    # Global
    plt.figure()
    plt.ylabel('Y (mm)')
    plt.xlabel('X (mm)')
    plt.axis('equal')
    plt.plot([frames[GLOBAL].position[0], frames[GLOBAL].position[0] + 50],
             [frames[GLOBAL].position[1], frames[GLOBAL].position[1]],
             '-r')
    plt.plot([frames[GLOBAL].position[0], frames[GLOBAL].position[0]],
             [frames[GLOBAL].position[1], frames[GLOBAL].position[1] + 50],
             '-g')

    scale = 50
    for name, frame in frames.items():
        if name != GLOBAL:
            rotation = frame.rotation
            plt.plot([frame.position[0], frame.position[0] + scale * rotation[0][0]],
                     [frame.position[1], frame.position[1] + scale * rotation[1][0]],
                     '-r')
            plt.plot([frame.position[0], frame.position[0] + scale * rotation[0][1]],
                     [frame.position[1], frame.position[1] + scale * rotation[1][1]],
                     '-g')

    # Silvia
    # plt.plot([frames[SILVIA2ROBOT].ref_point[0]],
    #          [frames[SILVIA2ROBOT].ref_point[1]],
    #          'ob')
    plt.plot([silvia_robot_point[0]],
             [silvia_robot_point[1]],
             '+', color='orange')

    # Grinder
    # plt.plot([frames[GRINDER2ROBOT].ref_point[0]],
    #          [frames[GRINDER2ROBOT].ref_point[1]],
    #          'ob')
    plt.plot([grinder_robot_point[0]],
             [grinder_robot_point[1]],
             '+', color='orange')

    # Cross
    # plt.plot([frames[cross2ROBOT].ref_point[0]],
    #          [frames[cross2ROBOT].ref_point[1]],
    #          'ob')
    plt.plot([cross_robot_point[0]],
             [cross_robot_point[1]],
             '+', color='orange')
    plt.show()


def main():
    filename = 'reference_frames.csv'
    frames = read_frames(filename)

    print(frames[GLOBAL + SILVIA].transform)
    print(frames[GLOBAL + GRINDER].transform)
    print(frames[GLOBAL + CROSS].transform)
    print(frames[GLOBAL + CUP].transform)

    silvia_frame_point = np.array([0, 218, 0, 1])
    silvia_robot_point = np.matmul(frames[GLOBAL + SILVIA].transform_np, silvia_frame_point)
    grinder_frame_point = np.array([157.61, 0, -250.45, 1])
    grinder_robot_point = np.matmul(frames[GLOBAL + GRINDER].transform_np, grinder_frame_point)
    cross_frame_point = np.array([-80, 0, -55, 1])
    cross_robot_point = np.matmul(frames[GLOBAL + CROSS].transform_np, cross_frame_point)

    test_points = [silvia_robot_point, grinder_robot_point, cross_robot_point]
    plot_scene(frames, test_points)


if __name__ == '__main__':
    main()
