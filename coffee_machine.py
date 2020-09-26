# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import robolink as rl  # RoboDK API
import robodk as rdk  # Robot toolbox
import datetime
from reference_frames import *

COMMAND_WAIT = 15
STRIP = 10

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
        self.log(datetime.datetime.now().ctime())

    def log(self, message):
        self.log_file.write(message + '\n')

    def close_log(self):
        self.log_file.close()

    def MoveJ(self, matrix, pos=""):
        self.robot.MoveJ(matrix)
        self.log("Joint move to new position: {}".format(pos))
        # self.log(matrix.ToString())

    def MoveL(self, matrix, pos=""):
        self.robot.MoveL(matrix)
        self.log("Linear move to new position: {}".format(pos))

    def tool_mount(self, name, pickup=True, location=STAND):
        if name == GRINDER:
            mount = GRINDERMOUNT
            func = GRINDERFUNC
        elif name == FILTER:
            mount = FILTERMOUNT
            func = FILTERFUNC
        else:
            mount = CUPMOUNT
            func = CUPFUNC

        if location == STAND:
            self.MoveJ(self.joint_angles[mount], mount)
            self.MoveJ(self.frames[GLOBAL + mount], mount)

        operation = ATTACH if pickup else DETACH
        name = func + operation + " (" + location.capitalize() + ")"
        self.log("Tool Mount Operation - Tool: {}, Operation:{}".format(name, operation))
        self.RDK.RunProgram(name, True)

    def cup_tool(self, operation):
        name = CUPFUNC + operation
        self.log("Cup Tool Operation - Operation:{}".format(operation))
        self.RDK.RunProgram(name, True)

    def insert_filter_grinder(self):
        self.log('')
        self.log(STRIP * '-' + ' Insert filter in grinder ' + '-' * STRIP)
        global2ball = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + BALL]  # * rdk.roty(0.1309)
        filter_over_ball = rdk.transl(0, 0, 50) * global2ball * self.frames[FILTER + TOOL] \
                           * self.frames[TOOL + TCP]
        tool_on_ball = global2ball * self.frames[BALL + TOOL] * self.frames[TOOL + TCP] * rdk.roty(0.1309)

        self.tool_mount(FILTER, True)
        # self.MoveJ(filter_over_ball)
        self.MoveJ(self.joint_angles['filterentry'], 'filter entry')
        # self.MoveJ(tool_on_ball)
        self.MoveJ(self.joint_angles['filterinsert'], 'filter insert')
        self.tool_mount(FILTER, False, GRINDER)
        self.MoveJ(self.frames[HOME], HOME)

    def cup_from_stack(self):
        self.log('')
        self.log(STRIP * '-' + ' Get cup from stack ' + '-' * STRIP)
        self.MoveJ(self.frames[HOME], HOME)
        self.tool_mount(CUP, True)
        self.frames[CUP + TOOL] = self.frames[TOOL + CUP].inv()
        cup_pickup_matrix = self.frames[GLOBAL + CUPSTACK] * self.frames[CUPSTACK + CUP] \
                            * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        starting_joint = [-58.899645, -62.825624, -101.410750, -195.763624, -58.899645, -40.000000]
        self.MoveL(starting_joint, 'Move to stack')  # Avoid other tools
        self.MoveJ(starting_joint, 'Get correct orientation')  # get correct orientation
        self.MoveL(self.joint_angles[CUPSTACK + 'entry'], CUPSTACK + 'entry')
        self.cup_tool(OPEN)

        self.MoveL(cup_pickup_matrix, CUPSTACK)
        self.cup_tool(CLOSE)

        remove_cup = rdk.transl(0, 0, 400) * cup_pickup_matrix
        # rotate_cup = remove_cup * rdk.rotz(3.141592653589793)
        self.MoveL(remove_cup, 'Remove cup')
        # self.MoveJ(remove_cup * rdk.rotz(3.141592653589793))
        rotated_cup = [-67.086904, -74.379835, -122.188350, 16.568185, 67.086904, -40.000000]
        self.MoveJ(rotated_cup, 'Rotate cup')

    def turn_on_coffee(self):
        self.log('')
        self.log(STRIP * '-' + ' Turn on coffee machine ' + '-' * STRIP)
        # Pickup filter
        self.MoveJ(self.frames[HOME])
        self.tool_mount(GRINDER, True)
        # self.MoveJ(self.joint_angles[GRINDERMOUNT], GRINDERMOUNT)

        # Move to button
        self.frames[TOOL + TCP] = self.frames[TCP + TOOL].inv()
        global2end = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + SILVIAPOWERBUTTON] \
                     * self.frames[TOOL + TCP] * rdk.transl(0, 0, -102.82)
        intermediate_point = rdk.transl(120, 100, 0) * global2end

        self.MoveJ(intermediate_point, 'Avoid tools')
        self.MoveJ(global2end, 'Move to button')
        push = global2end * rdk.transl(0, 0, 6)
        # global2end = frames[GLOBAL+SILVIA] * frames[SILVIA+SILVIAPOWERBUTTONPUSH] * frames[TOOL+TCP]
        self.MoveL(push, 'Push button')
        # move back
        self.MoveJ(global2end, 'Release')
        self.MoveJ(intermediate_point, 'Avoid tools')
        self.tool_mount(GRINDER, False)
        self.MoveJ(self.frames[HOME], HOME)


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
    machine.frames[BALL + TOOL] = machine.frames[TOOL + BALL].inv()
    machine.frames[FILTER + TOOL] = machine.frames[TOOL + FILTER].inv()

    # Run subprograms

    # machine.insert_filter_grinder()
    # machine.turn_on_coffee()
    # machine.cup_from_stack()

    machine.close_log()


main()
