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
PI = HALFPI * 2

GRINDERFUNC = "Grinder Tool"
FILTERFUNC = "Portafilter Tool"
CUPFUNC = "Cup Tool"
ATTACH = " Attach"
DETACH = " Detach"
STAND = "Stand"
OPEN = " Open"
CLOSE = " Close"
HOME = "Home"


class CoffeeMachine(object):

    def __init__(self, robot, master_tool, RDK, frames, joint_angles, log_filename="~/log.txt"):
        self.robot = robot
        self.master_tool = master_tool
        self.frames = frames
        self.joint_angles = joint_angles
        self.RDK = RDK
        self.log_filename = log_filename
        self.log_file = open(self.log_filename, "a")
        self.log("\n\n")
        self.log(datetime.datetime.now().ctime())

    def log(self, message):
        self.log_file.write(message + "\n")

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
            # self.MoveJ(self.frames[GLOBAL + mount], mount)    # Not needed as joint angles already calculated.

        operation = ATTACH if pickup else DETACH
        name = func + operation + " (" + location.capitalize() + ")"
        self.log("Tool Mount Operation - Tool: {}, Operation:{}".format(name, operation))
        self.RDK.RunProgram(name, True)
        if func == CUPFUNC:
            self.robot.setPoseTool(self.master_tool)

    def cup_tool(self, operation):
        name = CUPFUNC + operation
        self.log("Cup Tool Operation - Operation:{}".format(operation))
        self.RDK.RunProgram(name, True)

    def insert_filter_grinder(self):
        self.log("\n" + STRIP * "-" + " Insert filter in grinder " + "-" * STRIP)
        self.MoveJ(self.frames[HOME], HOME)

        # global2ball = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + BALL]  # * rdk.roty(0.1309)
        self.frames[GLOBAL+FILTER+ENTRY] = self.frames[GLOBAL+FILTER] * self.frames[TCP + TOOL] * \
            self.frames[TOOL + FILTER] * rdk.roty(-0.04) * rdk.transl(-2, 0, 0) \
            * self.frames[FILTER + TOOL] * rdk.transl(0, 0, 4) * self.frames[TOOL + TCP]
        # filter_over_ball = rdk.transl(0, 0, 60) * global2ball * rdk.roty(-0.1) * self.frames[FILTER + TOOL] \
        #     * self.frames[TOOL + TCP]

        self.tool_mount(FILTER, True)
        self.MoveJ(rdk.rotz(HALFPI) * self.frames[GLOBAL + FILTERMOUNT], "Intermediate point")
        self.MoveJ(self.joint_angles[FILTER + ENTRY], "Filter entry point")
        # self.MoveJ(filter_over_ball)
        self.MoveL(self.frames[GLOBAL + FILTER + ENTRY], "Insert filter")
        self.tool_mount(FILTER, False, GRINDER)
        self.MoveJ(self.joint_angles[FILTER + ENTRY], "Filter entry point")

    def turn_on_grinder(self):
        self.log("\n" + STRIP * "-" + " Turn on grinder " + "-" * STRIP)
        # self.MoveJ(self.joint_angles[FILTER + ENTRY], "Filter entry point")   # For testing collisions
        self.tool_mount(GRINDER, True)
        # self.MoveJ(self.joint_angles[GRINDERMOUNT], GRINDERMOUNT)  # For testing

        # Move to button
        global2on = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + GRINDERPOWERON] \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        release = global2on * rdk.transl(0, 0, -10)
        push = global2on * rdk.transl(0, 0, 7)
        intermediate_point = rdk.rotz(0.9) * self.frames[GLOBAL + GRINDERMOUNT]

        self.MoveJ(intermediate_point, "Avoid silvia and cups")
        self.MoveJ(self.joint_angles[GRINDERPOWERON], "Move to on button")
        self.MoveJ(release, "Move to on button")
        self.MoveL(push, "Push on button")
        self.MoveJ(release, "Release on button")
        rdk.pause(3)

        # Move back
        global2off = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + GRINDERPOWEROFF] \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        release = global2off * rdk.transl(0, 0, -10)
        push = global2off * rdk.transl(0, 0, 7)
        intermediate_point = rdk.transl(-10, -10, 30) * release

        self.MoveJ(release, "Move to off button")
        self.MoveL(push, "Push off button")
        self.MoveJ(release, "Release off button")
        self.MoveJ(intermediate_point, "Move away from grinder ready for lever movement")

    def pull_lever_multiple(self, n_pulls):
        self.log("\n" + STRIP * "-" + " Pull lever " + "-" * STRIP)

        global2start = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + LEVER] * rdk.transl(-5, -20, 0) \
            * self.frames[PULLER + TOOL] * self.frames[TOOL + TCP]
        mid_pull = global2start * rdk.transl(0, 0, -50)
        angle = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + LEVER] * rdk.transl(-5, -20, 0) \
            * rdk.transl(0, 0, -50) * rdk.roty(0.436332) * self.frames[PULLER + TOOL]
        end_pull = angle * rdk.transl(0, 0, -50) * self.frames[TOOL + TCP]
        exit_pos = angle * rdk.transl(0, 0, -50) * rdk.transl(50, -50, 0) * self.frames[TOOL + TCP]

        def pull(machine):
            machine.MoveJ(global2start, "Move to lever")
            machine.MoveJ(mid_pull, "Pull grinder lever")
            machine.MoveJ(angle * self.frames[TOOL + TCP], "Change angle")
            machine.MoveJ(end_pull, "Pull grinder lever")

        def release(machine):
            machine.MoveJ(angle * self.frames[TOOL + TCP], "Change angle")
            machine.MoveJ(mid_pull, "Pull grinder lever")
            machine.MoveJ(global2start, "Move to lever")

        self.MoveJ(self.joint_angles[GRINDER + LEVER], "Correct joint angles")
        for i in range(n_pulls):
            self.log("Lever pull # " + str(i))
            pull(self)
            if i < n_pulls - 1:
                release(self)

        self.MoveJ(exit_pos, "Release lever")
        self.tool_mount(GRINDER, False)
        # self.MoveJ(self.frames[HOME])

    def scrape_filter(self, scraper_height):
        self.log("\n" + STRIP * "-" + " Scrape coffee from filter " + "-" * STRIP)
        # self.MoveJ(self.frames[HOME])

        self.MoveJ(self.joint_angles[FILTER + ENTRY], "filter entry")
        self.tool_mount(FILTER, True, GRINDER)

        lift_off_ball = self.frames[GLOBAL + FILTER + ENTRY]
        pull_out = lift_off_ball * rdk.transl(0, 0, -70)
        # level = pull_out * self.frames[TCP + TOOL] * rdk.roty(0.1) \
        #     * self.frames[TOOL + TCP]

        self.MoveL(lift_off_ball, "Lift off ball")
        self.MoveL(pull_out, "Pull out filter")
        # self.MoveJ(level, "Level out filter")

        global2scraper = self.frames[GLOBAL + CROSS] * self.frames[CROSS + SCRAPER] * rdk.transl(scraper_height, 0, 0)
        start = global2scraper * rdk.transl(0, 0, -60) * self.frames[SCRAPER + FILTER] \
            * self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]
        end = global2scraper * rdk.transl(0, 0, 40) * self.frames[SCRAPER + FILTER] * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]

        self.MoveJ(start, "Scraper start")
        self.MoveL(end, "Push through scraper")
        self.MoveL(start, "Pull through scraper")

    def tamp_filter(self, depth):
        self.log("\n" + STRIP * "-" + " Tamp coffee filter " + "-" * STRIP)

        global2tamper = self.frames[GLOBAL + CROSS] * self.frames[CROSS + TAMPER]
        intermediate = global2tamper * rdk.transl(0, 0, -100) * self.frames[TAMPER + FILTER] * \
            self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]
        start = global2tamper * self.frames[TAMPER + FILTER] * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]

        end = global2tamper * self.frames[SCRAPER + FILTER] * rdk.transl(depth, 0, 0) * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]

        # self.MoveJ(intermediate, "Move to tamper")
        self.MoveJ(self.joint_angles[TAMPER + ENTRY], "Move to tamper")
        self.MoveJ(start, "Tamper start")
        self.MoveL(end, "Compress coffee")
        self.MoveL(start, "Lower filter")
        self.MoveJ(intermediate, "Remove from tamper")

    def insert_filter_silvia(self):
        self.log("\n" + STRIP * "-" + " Move filter to silvia " + "-" * STRIP)
        self.MoveJ(self.joint_angles[TAMPER + ENTRY], "Move to tamper")  # For testing
        intermediate = rdk.rotz(-1.5) * self.frames[GLOBAL + CROSS] * self.frames[CROSS + TAMPER] * \
            rdk.transl(70, 0, -100) * self.frames[TAMPER + FILTER] * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]
        entry = self.frames[GLOBAL+SILVIA] * rdk.transl(0, -100, -170) * self.frames[SILVIA+FILTER] \
            * self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]

        self.MoveJ(intermediate)
        self.MoveL(entry, "Filter to silvia")
        rdk.pause(15)   # For Rodney to remove filter tool and insert into machine

        # avoid_silvia = rdk.rotz(HALFPI) * self.frames[GLOBAL + CUPMOUNT]
        avoid_silvia_joints = [-88.986265, -77.001952, -78.888716, -114.109332, 90.000000, -178.986265]
        # self.MoveJ(avoid_silvia, "Avoid silvia")
        self.MoveJ(avoid_silvia_joints, "Avoid silvia")
        self.MoveJ(self.joint_angles[CUPMOUNT])

    def cup_from_stack(self):
        self.log("\n" + STRIP * "-" + " Get cup from stack " + "-" * STRIP)
        self.tool_mount(CUP, True)
        # self.MoveJ(self.joint_angles[CUPMOUNT])   # For testing

        cup_pickup_matrix = self.frames[GLOBAL + CUPSTACK] * self.frames[CUPSTACK + CUP] \
            * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        rotate_90 = rdk.rotz(HALFPI) * self.frames[GLOBAL+CUPMOUNT]
        intermediate = rdk.transl(0, 0, 150) * self.frames[GLOBAL + CUPSTACK] * self.frames[CUPSTACK + CUP] \
            * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        self.MoveJ(rotate_90, "Move to stack")  # Avoid other tools
        self.MoveJ(self.joint_angles[CUPSTACK + ENTRY], "Move to correct orientation")  # get correct orientation
        self.MoveJ(intermediate)

        before_cup = rdk.transl(0, 100, 0) * cup_pickup_matrix
        self.MoveL(before_cup, "Move to cup level")
        self.cup_tool(OPEN)

        self.MoveL(cup_pickup_matrix, "Slide to cup edge")
        self.cup_tool(CLOSE)

        remove_cup = rdk.transl(0, 0, 300) * cup_pickup_matrix
        self.MoveL(remove_cup, "Remove cup")
        self.MoveJ(self.joint_angles["rotated" + CUP], "Rotate cup")

    def place_cup(self, height):
        self.log('\n' + STRIP * '-' + ' Cup to Silvia ' + '-' * STRIP)

        end_point = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 7, 0) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        inter = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 15, -100) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        out = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 50, -150) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        # self.MoveJ(self.joint_angles['cupentry'], 'CupEntry')
        self.MoveJ(out)
        self.MoveJ(inter)
        self.MoveJ(end_point, 'Cup entry')
        self.cup_tool(OPEN)

        self.MoveJ(inter)
        self.cup_tool(CLOSE)
        self.MoveJ(out)
        self.tool_mount(CUP, False)

    def turn_on_silvia(self, time):
        self.log("\n" + STRIP * "-" + " Turn on coffee machine " + "-" * STRIP)

        self.tool_mount(GRINDER, True)
        # self.MoveJ(self.joint_angles[GRINDERMOUNT], GRINDERMOUNT)

        # Move to button
        on = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + SILVIAPOWERON] \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        off = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + SILVIAPOWEROFF] \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        intermediate_point = rdk.transl(120, 100, 0) * on
        self.MoveJ(intermediate_point, "Avoid tools")
        self.MoveJ(on, "Move to button")
        pushOn = on * rdk.transl(0, 0, 6)
        self.MoveL(pushOn, "Push button")
        self.MoveJ(on, "Release")

        rdk.pause(time)

        self.MoveJ(off, "Move to off")
        pushOff = off * rdk.transl(0, 0, 6)
        self.MoveL(pushOff, "Push button")
        self.MoveJ(off, "Release")
        self.MoveJ(intermediate_point, "Avoid tools")
        self.tool_mount(GRINDER, False)

    def pickup_coffee(self, height):
        self.log('\n' + STRIP * '-' + ' Cup to Rodney ' + '-' * STRIP)
        self.tool_mount(CUP, True)
        # self.MoveJ(self.joint_angles[CUPMOUNT])  # For testing

        end_point = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 7, 0) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        inter = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 15, -100) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        out = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 60, -180) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        self.MoveJ(out, 'Cup entry')
        self.MoveJ(inter)
        self.cup_tool(OPEN)
        self.MoveJ(end_point)
        self.cup_tool(CLOSE)
        self.MoveJ(inter)
        self.MoveJ(out)

        up = rdk.transl(0, 0, 350) * out
        down = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP+"place"] \
            * rdk.transl(height-10, 0, 0) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        over_silvia = rdk.transl(0, 0, 50) * down

        self.MoveL(up)
        self.MoveJ(over_silvia)
        self.MoveJ(down)
        self.cup_tool(OPEN)


def main():
    # Read transformation matrices from file
    frame_filename = "reference_frames.csv"
    joint_filename = "joint_angles.csv"
    logfile = "output.txt"
    frames = read_frames(frame_filename)
    joint_angles = read_joint_angles(joint_filename)

    # Read frames from station
    RDK = rl.Robolink()
    robot = RDK.Item("UR5")
    world_frame = RDK.Item("UR5 Base")
    home = RDK.Item("Home")  # existing target in station
    master_tool = RDK.Item("Master Tool")
    robot.setPoseFrame(world_frame)
    robot.setPoseTool(master_tool)

    machine = CoffeeMachine(robot, master_tool, RDK, frames, joint_angles, logfile)
    machine.frames[HOME] = home
    machine.frames[TOOL + TCP] = frames[TCP + TOOL].inv()
    machine.frames[BALL + TOOL] = machine.frames[TOOL + BALL].inv()
    machine.frames[FILTER + TOOL] = machine.frames[TOOL + FILTER].inv()
    machine.frames[PUSHER + TOOL] = machine.frames[TOOL + PUSHER].inv()
    machine.frames[PULLER + TOOL] = machine.frames[TOOL + PULLER].inv()
    machine.frames[CUP + TOOL] = machine.frames[TOOL + CUP].inv()

    # Run subprograms

    N = 3  # amount of times leaver needs to be pulled
    height = 98  # cup height
    time = 12    # TODO: Set to 12s for actual test
    scraper_height = 8  # TODO: think works but not sure
    tamp_height = 15  # TODO: Test
    machine.robot.setPoseTool(machine.master_tool)

    # machine.insert_filter_grinder()
    # machine.turn_on_grinder()
    # machine.pull_lever_multiple(N)
    # machine.scrape_filter(scraper_height)
    # machine.tamp_filter(tamp_height)
    # machine.insert_filter_silvia()
    # machine.cup_from_stack()
    # machine.place_cup(height)
    # machine.turn_on_silvia(time)
    machine.pickup_coffee(height)


main()
