# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import robolink as rl  # RoboDK API
import robodk as rdk  # Robot toolbox
# import numpy as np
from reference_frames import *

COMMAND_WAIT = 10

GRINDERFUNC = 'Grinder Tool'
FILTERFUNC = 'Portafilter Tool'
CUPFUNC = 'Cup Tool'
ATTACH = ' Attach'
DETACH = ' Detach'
HOME = 'Home'


class CoffeeMachine(object):

    def __init__(self, robot, RDK, frames, joint_angles):
        self.robot = robot
        self.frames = frames
        self.joint_angles = joint_angles
        self.functions = {}
        self.RDK = RDK

    def add_function(self, name):
        self.functions[name] = RDK.Item(name)

    def filtermount(self, pickup=True):
        self.robot.MoveJ(joint_angles[GLOBAL + FILTERMOUNT])
        self.robot.MoveJ(frames[GLOBAL + FILTERMOUNT])
        name = FILTERFUNC + (ATTACH if pickup else DETACH)
        self.functions[name].RunCode(name)
        rdk.pause(COMMAND_WAIT)


def main():
    # Read transformation matrices from file
    frame_filename = 'reference_frames.csv'
    joint_filename = 'joint_angles.csv'
    frames = read_frames(frame_filename)
    joint_angles = read_joint_angles(joint_filename)

    # Read frames from station
    RDK = rl.Robolink()
    robot = RDK.Item('UR5')
    world_frame = RDK.Item('UR5 Base')
    home = RDK.Item('Home')  # existing target in station
    robot.setPoseFrame(world_frame)
    robot.setPoseTool(robot.PoseTool())

    machine = CoffeeMachine(robot, RDK, frames, joint_angles)
    machine.frames[HOME] = home

    # Read subprograms
    grinder_tool_attach = RDK.Item('Grinder Tool Attach')
    grinder_tool_detach = RDK.Item('Grinder Tool Detach')
    filter_tool_attach = RDK.Item('Portafilter Tool Attach')
    filter_tool_detach = RDK.Item('Portafilter Tool Detach')
    cup_tool_attach = RDK.Item('Cup Tool Attach')
    cup_tool_detach = RDK.Item('Cup Tool Detach')

    # Pickup filter
    robot.MoveJ(home)
    # robot.MoveJ(joint_angles[GLOBAL + FILTERMOUNT])
    # robot.MoveJ(frames[GLOBAL + FILTERMOUNT])
    # filter_tool_attach.RunCode(filter_tool_attach)
    # # filter_tool_detach.RunCode(filter_tool_detach)
    # rdk.pause(COMMAND_WAIT)
    machine.filtermount(True)
    robot.MoveJ(home)
    machine.filtermount(False)

    # Move to ball
    frames[BALL+TOOL] = frames[TOOL+BALL].inv()
    frames[TOOL+TCP] = frames[TCP+TOOL].inv()
    frames[FILTER+TOOL] = frames[TOOL+FILTER].inv()
    global2end = frames[GLOBAL+GRINDER] * frames[GRINDER+BALL] * frames[BALL+TOOL] * frames[TOOL+TCP]
    # robot.MoveJ(global2end)
    # rdk.pause(COMMAND_WAIT)
    # robot.MoveJ(home)


main()