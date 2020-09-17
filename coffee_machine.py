# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import robolink as rl  # RoboDK API
import robodk as rdk  # Robot toolbox
# import numpy as np
from reference_frames import *

COMMAND_WAIT = 15

GRINDERFUNC = 'Grinder Tool'
FILTERFUNC = 'Portafilter Tool'
CUPFUNC = 'Cup Tool'
ATTACH = ' Attach'
DETACH = ' Detach'
OPEN = ' Open'
CLOSE = ' Close'
HOME = 'Home'


class CoffeeMachine(object):

    def __init__(self, robot, RDK, frames, joint_angles, log_file='~/log.txt'):
        self.robot = robot
        self.frames = frames
        self.joint_angles = joint_angles
        self.functions = {}
        self.RDK = RDK
        self.log_filename = log_file
        self.log_file = open(logfile, 'w')

    def log(self, message):
        self.log_file.write(message)

    def close_log(self):
        close(self.log_file)

    def add_function(self, name):
        self.functions[name] = self.RDK.Item(name)
        self.log("Function '{}' added".format(name))

    def MoveJ(self, matrix):
        self.robot.MoveJ(matrix)
        self.log("Moving to new position")
        self.log(matrix)

    def tool_mount(self, name, pickup=True):
        if name == GRINDER:
            mount = GRINDERMOUNT
            func = GRINDERFUNC
        elif name == FILTER:
            mount = FILTERMOUNT
            func = FILTERFUNC
        elif name == CUP:
            mount = CUPMOUNT
            func = CUPFUNC

        self.robot.MoveJ(self.joint_angles[GLOBAL + mount])
        self.robot.MoveJ(self.frames[GLOBAL + mount])
        operation = ATTACH if pickup else DETACH
        name = func + operation
        self.functions[name].RunCode(self.functions[name], True)
        self.log("Tool Mount Operation - Tool: {}, Operation: {}".format(name, operation))


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

    machine.add_function(GRINDERFUNC+ATTACH)
    machine.add_function(GRINDERFUNC+DETACH)
    machine.add_function(FILTERFUNC+ATTACH)
    machine.add_function(FILTERFUNC+DETACH)
    machine.add_function(CUPFUNC+ATTACH)
    machine.add_function(CUPFUNC+DETACH)
    machine.add_function(CUPFUNC+OPEN)
    machine.add_function(CUPFUNC+CLOSE)

    # Pickup filter
    robot.MoveJ(home)
    machine.tool_mount(GRINDER, True)
    machine.tool_mount(GRINDER, False)
    robot.MoveJ(home)
    machine.tool_mount(FILTER, True)
    machine.tool_mount(FILTER, False)
    robot.MoveJ(home)
    machine.tool_mount(CUP, True)
    machine.tool_mount(CUP, False)
    robot.MoveJ(home)

    # Move to ball
    frames[BALL + TOOL] = frames[TOOL + BALL].inv()
    frames[TOOL + TCP] = frames[TCP + TOOL].inv()
    frames[FILTER + TOOL] = frames[TOOL + FILTER].inv()
    global2end = frames[GLOBAL + GRINDER] * frames[GRINDER + BALL] * frames[BALL + TOOL] * frames[TOOL + TCP]
    robot.MoveJ(machine.joint_angles[GLOBAL+'filterinsert'])
    robot.MoveJ(global2end)
    # robot.MoveJ(home)

    machine.close_log()


main()
