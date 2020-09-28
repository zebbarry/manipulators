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
HALFPI = 1.570796326794897

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
        self.log('\n' + STRIP * '-' + ' Insert filter in grinder ' + '-' * STRIP)
        self.MoveJ(self.frames[HOME], HOME)
        global2ball = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + BALL]  # * rdk.roty(0.1309)
        filter_over_ball = rdk.transl(0, 0, 50) * global2ball * self.frames[FILTER + TOOL] \
                           * self.frames[TOOL + TCP]
        tool_on_ball = global2ball * self.frames[BALL + TOOL] * self.frames[TOOL + TCP] * rdk.roty(0.1309)

        self.tool_mount(FILTER, True)
        # self.MoveJ(filter_over_ball)
        self.MoveJ(rdk.rotz(HALFPI) * self.frames[GLOBAL + FILTERMOUNT])
        self.MoveJ(self.joint_angles['filterentry'], 'filter entry')
        # self.MoveJ(tool_on_ball)
        self.MoveJ(self.frames[GLOBAL + FILTER])
        # self.MoveJ(self.joint_angles['filterinsert'], 'filter insert')
        self.tool_mount(FILTER, False, GRINDER)
        self.MoveJ(self.joint_angles['filterentry'], 'filter entry')

    def cup_from_stack(self):
        self.log('\n' + STRIP * '-' + ' Get cup from stack ' + '-' * STRIP)
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

    def turn_on_silvia(self):
        self.log('\n' + STRIP * '-' + ' Turn on coffee machine ' + '-' * STRIP)
        self.MoveJ(self.frames[HOME])
        self.tool_mount(GRINDER, True)
        # self.MoveJ(self.joint_angles[GRINDERMOUNT], GRINDERMOUNT)

        # Move to button
        global2end = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + SILVIAPOWER] \
                     * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        intermediate_point = rdk.transl(120, 100, 0) * global2end

        self.MoveJ(intermediate_point, 'Avoid tools')
        self.MoveJ(global2end, 'Move to button')
        push = global2end * rdk.transl(0, 0, 6)
        self.MoveL(push, 'Push button')
        # move back
        self.MoveJ(global2end, 'Release')
        self.MoveJ(intermediate_point, 'Avoid tools')
        self.tool_mount(GRINDER, False)
        self.MoveJ(self.frames[HOME], HOME)

    def turn_on_grinder(self):
        self.log('\n' + STRIP * '-' + ' Turn on grinder ' + '-' * STRIP)
        self.MoveJ(self.frames[HOME])
        self.tool_mount(GRINDER, True)
        # self.MoveJ(self.joint_angles[GRINDERMOUNT], GRINDERMOUNT)

        # Move to button
        global2on = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + GRINDERPOWERON] \
                    * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        release = global2on * rdk.transl(0, 0, -10)
        push = global2on * rdk.transl(0, 0, 7)
        # intermediate_point = rdk.rotz() * self.frames[GLOBAL + GRINDERMOUNT]

        # self.MoveJ(intermediate_point, 'Avoid tools')
        self.MoveJ(self.joint_angles[GRINDERPOWERON], 'Move to on button')
        self.MoveJ(release, 'Move to on button')
        self.MoveL(push, 'Push on button')
        self.MoveJ(release, 'Release on button')
        rdk.pause(3)

        # Move back
        global2off = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + GRINDERPOWEROFF] \
                     * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        release = global2off * rdk.transl(0, 0, -10)
        push = global2off * rdk.transl(0, 0, 7)
        intermediate_point = rdk.transl(-10, -10, 30) * release

        self.MoveJ(release, 'Move to off button')
        self.MoveL(push, 'Push off button')
        self.MoveJ(release, 'Release off button')
        self.MoveJ(intermediate_point, 'Avoid grinder')

    def pull_lever(self):
        self.log('\n' + STRIP * '-' + ' Pull lever ' + '-' * STRIP)

        n = 3 # amount of times leaver needs to be pulled

        global2start = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + LEVER] * rdk.transl(-5, -20, 0) \
                       * self.frames[PULLER + TOOL] * self.frames[TOOL + TCP]
        mid_pull = global2start * rdk.transl(0, 0, -50)
        angle = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + LEVER] * rdk.transl(-5, -20, 0) \
                * rdk.transl(0, 0, -50) * rdk.roty(0.436332) * self.frames[PULLER + TOOL]
        end_pull = angle * rdk.transl(0, 0, -40) * self.frames[TOOL + TCP]
        release = angle * rdk.transl(0, 0, -40) * rdk.transl(50, -50, 0) * self.frames[TOOL + TCP]
        i = 0
        while i < n:

            self.MoveJ(self.joint_angles[GRINDER + LEVER], 'Correct joint angles')
            self.MoveJ(global2start, 'Move to lever')
            self.MoveJ(mid_pull, 'Pull grinder lever')
            self.MoveJ(angle * self.frames[TOOL + TCP], 'Change angle')
            self.MoveJ(end_pull, 'Pull grinder lever')
            self.MoveJ(release, 'Release lever')
            i++

        self.tool_mount(GRINDER, False)
        self.MoveJ(self.frames[HOME])

    def scrape_filter(self):
        self.log('\n' + STRIP * '-' + ' Scrape coffee from filter ' + '-' * STRIP)
        self.MoveJ(self.frames[HOME])

        self.MoveJ(self.joint_angles['filterentry'], 'filter entry')
        self.tool_mount(FILTER, True, GRINDER)

        lift_off_ball = self.robot.Pose() * self.frames[TCP + TOOL] * self.frames[TOOL + FILTER] * rdk.roty(-0.1) \
                        * rdk.transl(-2, 0, 0) * self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]
        pull_out = lift_off_ball * rdk.transl(0, 0, -70)
        level = pull_out * self.frames[TCP + TOOL] * rdk.roty(0.1) \
                * self.frames[TOOL + TCP]

        self.MoveL(lift_off_ball, 'Lift off ball')
        self.MoveL(pull_out, 'Pull out filter')
        self.MoveJ(level, 'Pull out filter')

        global2scraper = self.frames[GLOBAL + CROSS] * self.frames[CROSS + SCRAPER]
        start = global2scraper * rdk.transl(0, 0, -100) * self.frames[SCRAPER + FILTER] * self.frames[FILTER + TOOL] \
                * self.frames[TOOL + TCP]
        end = global2scraper * rdk.transl(0, 0, 50) * self.frames[SCRAPER + FILTER] * self.frames[FILTER + TOOL] \
              * self.frames[TOOL + TCP]

        self.MoveJ(start, 'Scraper start')
        self.MoveL(end, 'Scraper end')
        self.MoveL(start, 'Scraper start')

    def tamp_filter(self):
        self.log('\n' + STRIP * '-' + ' Tamp coffee filter ' + '-' * STRIP)

        global2tamper = self.frames[GLOBAL + CROSS] * self.frames[CROSS + TAMPER]
        intermediate = global2tamper * rdk.transl(0, 0, -100) * self.frames[TAMPER + FILTER] * self.frames[FILTER + TOOL] \
                * self.frames[TOOL + TCP]
        start = global2tamper * self.frames[TAMPER + FILTER] * self.frames[FILTER + TOOL] \
                * self.frames[TOOL + TCP]
        depth = 8
        end = global2tamper * self.frames[SCRAPER + FILTER] * rdk.transl(depth, 0, 0) * self.frames[FILTER + TOOL] \
                * self.frames[TOOL + TCP]

        self.MoveJ(intermediate, 'Move to tamper')
        self.MoveJ(start, 'Tamper start')
        self.MoveL(end, 'Compress coffee')
        self.MoveL(start, 'Lower filter')
        self.MoveJ(intermediate, 'Remove from tamper')


    def insert_filter_silvia(self):
        pass

    def select_coffee(self):
        pass

    def pickup_cup(self):
        pass


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
    machine.frames[PUSHER + TOOL] = machine.frames[TOOL + PUSHER].inv()
    machine.frames[PULLER + TOOL] = machine.frames[TOOL + PULLER].inv()

    # Run subprograms

    # machine.insert_filter_grinder()
    # machine.turn_on_grinder()
    machine.pull_lever()
    # machine.scrape_filter()
    # machine.tamp_filter()
    # machine.turn_on_silvia()
    # machine.cup_from_stack()

    machine.close_log()


main()
