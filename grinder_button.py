# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import robolink as rl  # RoboDK API
import robodk as rdk  # Robot toolbox
from reference_frames import *

COMMAND_WAIT = 15

GRINDERFUNC = 'Grinder Tool'
FILTERFUNC = 'Portafilter Tool'
CUPFUNC = 'Cup Tool'
ATTACH = ' Attach'
DETACH = ' Detach'
STAND = 'Stand'
OPEN = ' Open'
CLOSE = ' Close'
HOME = 'Home'


class CoffeeMachine(object):

    def __init__(self, robot, RDK, frames, joint_angles, log_filename='~/log.txt'):
        self.robot = robot
        self.frames = frames
        self.joint_angles = joint_angles
        self.RDK = RDK
        self.log_filename = log_filename
        self.log_file = open(self.log_filename, 'a')
        self.log("\n\n")

    def log(self, message):
        self.log_file.write(message)

    def close_log(self):
        self.log_file.close()

    def MoveJ(self, matrix, pos=""):
        self.robot.MoveJ(matrix)
        self.log("Joint move to new position: {}\n".format(pos))
        # self.log(matrix.ToString())

    def MoveL(self, matrix, pos=""):
        self.robot.MoveL(matrix)
        self.log("Linear move to new position: {}\n".format(pos))

    def tool_mount(self, name, pickup=True, location=STAND):
        if name == GRINDER:
            mount = GRINDERMOUNT
            func = GRINDERFUNC
        elif name == FILTER:
            mount = FILTERMOUNT
            func = FILTERFUNC
        else:  # name == CUP
            mount = CUPMOUNT
            func = CUPFUNC

        self.MoveJ(self.joint_angles[mount], mount)
        self.MoveJ(self.frames[GLOBAL + mount], mount)
        operation = ATTACH if pickup else DETACH
        name = func + operation + " (" + location.capitalize() + ")"
        self.log("Tool Mount Operation - Tool: {}, Operation:{}\n".format(name, operation))
        self.RDK.RunProgram(name, True)

    def cup_tool(self, operation):
        name = CUPFUNC + operation
        self.log("Cup Tool Operation - Operation:{}\n".format(operation))
        self.RDK.RunProgram(name, True)

    def cup_from_stack(self):
        self.MoveJ(self.frames[HOME], HOME)
        self.tool_mount(CUP, True)
        self.frames[CUP + TOOL] = self.frames[TOOL + CUP].inv()
        cup_pickup_matrix = self.frames[GLOBAL + CUPSTACK] * self.frames[CUPSTACK + CUP] \
                            * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        starting_joint = [-58.899645, -62.825624, -101.410750, -195.763624, -58.899645, -40.000000]
        self.MoveL(starting_joint)  # Avoid other tools
        self.MoveJ(starting_joint)  # get correct orientation
        self.MoveL(self.joint_angles[CUPSTACK + 'entry'], CUPSTACK + 'entry')
        self.cup_tool(OPEN)

        self.MoveL(cup_pickup_matrix, CUPSTACK)
        self.cup_tool(CLOSE)

        remove_cup = rdk.transl(0, 0, 400) * cup_pickup_matrix
        # rotate_cup = remove_cup * rdk.rotz(3.141592653589793)
        self.MoveL(remove_cup)
        # self.MoveJ(remove_cup * rdk.rotz(3.141592653589793))
        self.MoveJ([-67.086904, -74.379835, -122.188350, 16.568185, 67.086904, -40.000000])
        self.cup_tool(OPEN)
        rdk.pause(COMMAND_WAIT)
        # self.cup_tool(CLOSE)
        # self.MoveJ(starting_joint)  # get correct orientation
        # self.MoveL(self.joint_angles[CUPMOUNT], CUPMOUNT)
        # self.tool_mount(CUP, False)

    def turn_on_coffee(self):
        # Pickup filter
        self.MoveJ(self.frames[HOME])
        self.tool_mount(GRINDER, True)
        # self.MoveJ(self.joint_angles[GRINDERMOUNT], GRINDERMOUNT)

        # Move to button
        self.frames[TOOL+TCP] = self.frames[TCP+TOOL].inv()
        global2end = self.frames[GLOBAL+SILVIA] * self.frames[SILVIA+SILVIAPOWERBUTTON] * self.frames[TOOL+TCP] * rdk.transl(0, 0, -102.82)
        intermediate_point = rdk.transl(120, 100, 0) * global2end
        # print(global2end)
        self.MoveJ(intermediate_point)
        self.MoveJ(global2end)
        push = global2end * rdk.transl(0, 0, 6)
        #global2end = frames[GLOBAL+SILVIA] * frames[SILVIA+SILVIAPOWERBUTTONPUSH] * frames[TOOL+TCP]
        self.MoveL(push)
         # move back
        self.MoveJ(global2end)
        self.MoveJ(intermediate_point)
        self.tool_mount(GRINDER, False)
        self.MoveJ(self.frames[HOME])



def main():
    # Read transformation matrices from file
    frame_filename = 'reference_frames.csv'
    joint_filename = 'joint_angles.csv'
    logfile = 'output.txt'
    frames = read_frames(frame_filename)
    joint_angles = read_joint_angles(joint_filename)

    # Read frames from station
    RDK = rl.Robolink()
    robot = RDK.Item('UR5')
    world_frame = RDK.Item('UR5 Base')
    home = RDK.Item('Home')  # existing target in station
    robot.setPoseFrame(world_frame)
    robot.setPoseTool(robot.PoseTool())

    machine = CoffeeMachine(robot, RDK, frames, joint_angles, logfile)
    machine.frames[HOME] = home
    machine.frames[TOOL + TCP] = frames[TCP + TOOL].inv()

    # Read subprograms

    # Pickup filter
    # machine.MoveJ(home)
    # machine.tool_mount(GRINDER, True)
    # machine.tool_mount(GRINDER, False)
    # machine.MoveJ(home)
    # machine.tool_mount(FILTER, True)
    # machine.tool_mount(FILTER, False)
    # machine.MoveJ(home)
    # machine.tool_mount(CUP, True)
    # machine.tool_mount(CUP, False)
    # machine.MoveJ(home)

    # Move to ball
    # machine.frames[BALL + TOOL] = frames[TOOL + BALL].inv()
    # machine.frames[FILTER + TOOL] = frames[TOOL + FILTER].inv()
    # global2end = machine.frames[GLOBAL + GRINDER] * machine.frames[GRINDER + BALL] \
    #              * machine.frames[BALL + TOOL] * machine.frames[TOOL + TCP]
    # machine.MoveJ(machine.joint_angles['filterinsert'])
    # machine.MoveJ(global2end)
    # machine.MoveJ(home)

    machine.turn_on_coffee()
    machine.cup_from_stack()

    machine.close_log()


main()
