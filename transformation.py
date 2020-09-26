import matplotlib.pyplot as plt
import robodk as rdk
from reference_frames import *


def plot_scene(frames, test_points):
    silvia_robot_point, \
    grinder_robot_point, \
    cross_robot_point = [point.tolist() for point in test_points]

    # Global
    plt.figure()
    plt.ylabel('Y (mm)')
    plt.xlabel('X (mm)')
    plt.axis('equal')
    plt.plot([frames[GLOBAL].Pos()[0], frames[GLOBAL].Pos()[0] + 50],
             [frames[GLOBAL].Pos()[1], frames[GLOBAL].Pos()[1]],
             '-r')
    plt.plot([frames[GLOBAL].Pos()[0], frames[GLOBAL].Pos()[0]],
             [frames[GLOBAL].Pos()[1], frames[GLOBAL].Pos()[1] + 50],
             '-g')

    scale = 50
    for name, frame in frames.items():
        if name != GLOBAL:
            rotation = frame.Rot33().Rows()
            plt.plot([frame.Pos()[0], frame.Pos()[0] + scale * rotation[0][0]],
                     [frame.Pos()[1], frame.Pos()[1] + scale * rotation[1][0]],
                     '-r')
            plt.plot([frame.Pos()[0], frame.Pos()[0] + scale * rotation[0][1]],
                     [frame.Pos()[1], frame.Pos()[1] + scale * rotation[1][1]],
                     '-g')

    plt.plot([silvia_robot_point[0]],
             [silvia_robot_point[1]],
             '+', color='orange')
    plt.plot([grinder_robot_point[0]],
             [grinder_robot_point[1]],
             '+', color='orange')
    plt.plot([cross_robot_point[0]],
             [cross_robot_point[1]],
             '+', color='orange')
    plt.plot([cross_robot_point[0]],
             [cross_robot_point[1]],
             '+', color='orange')
    plt.show()


def main():
    filename = 'reference_frames.csv'
    frames = read_frames(filename)

    all_true = True
    for name, frame in frames.items():
        print(name)
        print(frame)
        print("{} is homogenous: {}".format(name, frame.isHomogeneous()))
        if not frame.isHomogeneous():
            all_true = False
    print(all_true)

    silvia_frame_point = rdk.Mat([0, 218, 0, 1])
    silvia_robot_point = frames[GLOBAL + SILVIA] * silvia_frame_point
    grinder_frame_point = rdk.Mat([157.61, 0, -250.45, 1])
    grinder_robot_point = frames[GLOBAL + GRINDER] * grinder_frame_point
    cross_frame_point = rdk.Mat([-80, 0, -55, 1])
    cross_robot_point = frames[GLOBAL + CROSS] * cross_frame_point

    test_points = [silvia_robot_point, grinder_robot_point, cross_robot_point]
    plot_scene(frames, test_points)


if __name__ == '__main__':
    main()
