# Program for controlling a UR5 collaborative robot to make a coffee
# Read joint angle configurations and transformation matrices from separate CSV files
# Authors: Zeb Barry, Jack Zarifeh
# Date: 14th October 2020
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
import robolink as rl  # RoboDK API
import robodk as rdk  # Robot toolbox
import datetime

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
GLOBAL = "global"
SILVIA = "silvia"
GRINDER = "grinder"
CUPSTACK = "cupstack"
CUP = "cup"
CROSS = "cross"
TCP = "tcp"
TOOL = "tool"
PUSHER = "pusher"
PULLER = "puller"
LEVER = "lever"
GRINDERMOUNT = "grindermount"
FILTERMOUNT = "filtermount"
CUPMOUNT = "cupmount"
FILTER = "filter"
SCRAPER = "scraper"
TAMPER = "tamper"
BALL = "ball"
ENTRY = "entry"
SILVIAPOWERON = "silviapoweron"
SILVIAPOWEROFF = "silviapoweroff"
GRINDERPOWERON = "grinderpoweron"
GRINDERPOWEROFF = "grinderpoweroff"


def read_frames(filename):
    """ Read transformation matrices from CSV file and convert into dictionary of RoboDK matrices.
    filename: file path to CSV file
    output: dictionary of transformation matrices where key is a string
     composed of the first frame and second frame titles
    """
    # Open file
    file = open(filename, "r")
    lines = file.readlines()
    file.close()

    # Read each line of file and convert into RoboDK matrix
    frames = {}
    for line in lines:
        segments = line.rstrip().split(",")
        name = segments[0]
        values = [float(i) for i in segments[1:]]
        # CSV file is formatted as each row sequentially separated by ','
        transform = rdk.Mat([values[0:4],
                             values[4:8],
                             values[8:12],
                             values[12:16]])
        frames[name] = transform
    return frames


def read_joint_angles(filename):
    """ Read joint angles from CSV file and convert into dictionary of RoboDK matrices.
    filename: file path to CSV file
    output: dictionary of joint angles  where key describing the final pose
    """
    # Open file
    file = open(filename, "r")
    lines = file.readlines()
    file.close()

    # Read each line of file and convert into RoboDK matrix
    joint_angles = {}
    for line in lines:
        segments = line.rstrip().split(",")
        name = segments[0]
        values = [float(i) for i in segments[1:]]
        angles = rdk.Mat(values)
        joint_angles[name] = angles
    return joint_angles


class CoffeeMachine(object):
    """ Class definition for coffee machine object, contains all methods for completing tasks
    as well as logging results.
    """

    def __init__(self, robot, master_tool, RDK, frames, joint_angles, log_filename="~/log.txt"):
        self.robot = robot
        self.master_tool = master_tool
        self.frames = frames
        self.joint_angles = joint_angles
        self.RDK = RDK
        self.log_filename = log_filename

        # Open log file and write current date and time
        self.log_file = open(self.log_filename, "a")
        self.log("\n\n")
        self.log(datetime.datetime.now().ctime())

    def log(self, message):
        """ Write message to log file
        message: message as a string
        """
        self.log_file.write(message + "\n")

    def close_log(self):
        """ Close log file """
        self.log_file.close()

    def MoveJ(self, matrix, pos=""):
        """ Perform joint move to new position and write message to log
        matrix: Either transform matrix describing TCP in global frame or joint angle configuration
        pos: message to be written to log file describing movement
        """
        self.robot.MoveJ(matrix)
        self.log("Joint move to new position: {}".format(pos))

    def MoveL(self, matrix, pos=""):
        """ Perform linear move to new position and write message to log
        matrix: Either transform matrix describing TCP in global frame or joint angle configuration
        pos: message to be written to log file describing movement
        """
        self.robot.MoveL(matrix)
        self.log("Linear move to new position: {}".format(pos))

    def tool_mount(self, name, pickup=True, location=STAND):
        """ General function for moving to and attaching/detaching tools.
        name: name of tool as string
        pickup: boolean to determine whether to attach (True) or detach (False) tool
        location: location of tool as string, options are tool stand or grinder
        """
        # Determine which tool to use
        if name == GRINDER:
            mount = GRINDERMOUNT
            func = GRINDERFUNC
        elif name == FILTER:
            mount = FILTERMOUNT
            func = FILTERFUNC
        else:
            mount = CUPMOUNT
            func = CUPFUNC

        # If tool is at stand move to position, for grinder location tool must already be in position
        if location == STAND:
            self.MoveJ(self.joint_angles[mount], mount)
            # self.MoveJ(self.frames[GLOBAL + mount], mount)    # Not needed as joint angles already calculated.

        # Determine if attaching or detaching tool
        operation = ATTACH if pickup else DETACH
        # Create string function name and run command, logging output
        name = func + operation + " (" + location.capitalize() + ")"
        self.log("Tool Mount Operation - Tool: {}, Operation:{}".format(name, operation))
        self.RDK.RunProgram(name, True)
        # Cup tool occasionally has issues with defining reference frame so redefine as TCP
        if func == CUPFUNC:
            self.robot.setPoseTool(self.master_tool)

    def cup_tool(self, operation):
        """ Operate cup tool
        operation: String describing whether to open or close tool
        """
        name = CUPFUNC + operation
        self.log("Cup Tool Operation - Operation:{}".format(operation))
        self.RDK.RunProgram(name, True)

    def insert_filter_grinder(self):
        """" Insert the portafilter into the grinder machine """
        self.log("\n" + STRIP * "-" + " Insert filter in grinder " + "-" * STRIP)
        # Start in home position
        self.MoveJ(self.frames[HOME], HOME)

        # global2ball = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + BALL]
        # filter_over_ball = rdk.transl(0, 0, 60) * global2ball * rdk.roty(-0.1) * self.frames[FILTER + TOOL] \
        #     * self.frames[TOOL + TCP]

        # Calculate transform matrix for filter in holder of grinder
        self.frames[GLOBAL+FILTER+ENTRY] = self.frames[GLOBAL+FILTER] * self.frames[TCP + TOOL] * \
            self.frames[TOOL + FILTER] * rdk.roty(-0.04) * rdk.transl(-2, 0, 0) \
            * self.frames[FILTER + TOOL] * rdk.transl(0, 0, 4) * self.frames[TOOL + TCP]

        # Mount filter tool and insert into machine
        self.tool_mount(FILTER, True)
        self.MoveJ(rdk.rotz(HALFPI) * self.frames[GLOBAL + FILTERMOUNT], "Intermediate point")
        self.MoveJ(self.joint_angles[FILTER + ENTRY], "Filter entry point")
        # self.MoveJ(filter_over_ball) # Used to determine joint angles, not needed
        self.MoveL(self.frames[GLOBAL + FILTER + ENTRY], "Insert filter")
        # Detach tool in grinder and back away
        self.tool_mount(FILTER, False, GRINDER)
        self.MoveJ(self.joint_angles[FILTER + ENTRY], "Return to filter entry point")

    def turn_on_grinder(self):
        """ Turn on grinder """
        self.log("\n" + STRIP * "-" + " Turn on grinder " + "-" * STRIP)
        # Attach grinder tool
        self.tool_mount(GRINDER, True)

        # Calculate transform for grinder on button
        global2on = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + GRINDERPOWERON] * rdk.rotz(PI)\
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        release = global2on * rdk.transl(0, 0, -10)
        push = global2on * rdk.transl(0, 0, 7)
        intermediate_point = rdk.rotz(0.9) * self.frames[GLOBAL + GRINDERMOUNT]

        self.MoveJ(intermediate_point, "Avoid silvia and cups")
        self.MoveJ(self.joint_angles[GRINDERPOWERON], "Move to on button and ensure lever arm on left")
        self.MoveJ(release, "Move to on button")
        self.MoveL(push, "Push on button")
        self.MoveJ(release, "Release on button")
        # Wait for grinder to grind beans
        rdk.pause(3)

        # Calculate transform for grinder off button
        global2off = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + GRINDERPOWEROFF] * rdk.rotz(PI) \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        release = global2off * rdk.transl(0, 0, -10)
        push = global2off * rdk.transl(0, 0, 7)
        intermediate_point = rdk.transl(-50, -40, 30) * release

        self.MoveJ(release, "Move to off button")
        self.MoveL(push, "Push off button")
        self.MoveJ(release, "Release off button")
        self.MoveJ(intermediate_point, "Move away from grinder ready for lever movement")

    def pull_lever_multiple(self, n_pulls):
        """ Pull lever n_pulls times using grinder tool
        n_pulls: int describing number of times to pull lever
        """
        self.log("\n" + STRIP * "-" + " Pull lever " + "-" * STRIP)

        # Calculate transform for start position of lever
        global2start = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + LEVER] * rdk.transl(-5, -20, 0) \
            * self.frames[PULLER + TOOL] * self.frames[TOOL + TCP]
        # Determine mid point of bi-linear movement
        mid_pull = global2start * rdk.transl(0, 0, -50)
        # Adjust angle to allow rotation about the grinder
        angle = self.frames[GLOBAL + GRINDER] * self.frames[GRINDER + LEVER] * rdk.transl(-5, -20, 0) \
            * rdk.transl(0, 0, -50) * rdk.roty(0.436332) * self.frames[PULLER + TOOL]
        # Complete bi-linear pull
        end_pull = angle * rdk.transl(0, 0, -50) * self.frames[TOOL + TCP]
        exit_pos = angle * rdk.transl(0, 0, -50) * rdk.transl(50, -50, 0) * self.frames[TOOL + TCP]

        # Function definition for bi-linear pull movement of lever
        def pull(machine):
            machine.MoveJ(global2start, "Move to lever")
            machine.MoveJ(mid_pull, "Pull grinder lever")
            machine.MoveJ(angle * self.frames[TOOL + TCP], "Change angle")
            machine.MoveJ(end_pull, "Pull grinder lever")

        # Reverse bi-linear pull in preparation for another pull
        def release(machine):
            machine.MoveJ(angle * self.frames[TOOL + TCP], "Change angle")
            machine.MoveJ(mid_pull, "Pull grinder lever")
            machine.MoveJ(global2start, "Move to lever")

        self.MoveJ(self.joint_angles[GRINDER + LEVER], "Move to lever with correct joint angles")
        # Pull lever n_pulls times and release n_pulls-1 times
        for i in range(n_pulls):
            self.log("Lever pull # " + str(i))
            pull(self)
            if i < n_pulls - 1:
                release(self)

        self.MoveJ(exit_pos, "Release lever")
        # Remove grinder tool
        self.tool_mount(GRINDER, False)

    def scrape_filter(self, scraper_height):
        """ Remove filter from grinder and scrape off excess grinds using scraper
        scraper_height: int allowing tuning of vertical position of filter w.r.t scraper
        """
        self.log("\n" + STRIP * "-" + " Scrape coffee from filter " + "-" * STRIP)

        self.MoveJ(self.joint_angles[FILTER + ENTRY], "Move to filter at grinder entry point")
        # Attach portafilter tool at grinder
        self.tool_mount(FILTER, True, GRINDER)

        # Calculate transforms for removing portafilter tool
        lift_off_ball = self.frames[GLOBAL + FILTER + ENTRY]
        pull_out = lift_off_ball * rdk.transl(0, 0, -70)

        self.MoveL(lift_off_ball, "Lift filter off ball")
        self.MoveL(pull_out, "Pull out filter tool")

        # Positions for start and end of scraper movement. Dependant on scraper_height
        global2scraper = self.frames[GLOBAL + CROSS] * self.frames[CROSS + SCRAPER] * rdk.transl(scraper_height, 0, 0)
        start = global2scraper * rdk.transl(0, 0, -60) * self.frames[SCRAPER + FILTER] \
            * self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]
        end = global2scraper * rdk.transl(0, 0, 40) * self.frames[SCRAPER + FILTER] * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]

        self.MoveJ(start, "Scraper start")
        self.MoveL(end, "Push through scraper")
        self.MoveL(start, "Pull through scraper")

    def tamp_filter(self, depth):
        """ Tamp coffee in filter using portafilter tool
        depth: how deep to push the tamper into the filter, int
        """
        self.log("\n" + STRIP * "-" + " Tamp coffee filter " + "-" * STRIP)

        # Calculate transforms for positioning filter below tamper and tamping
        global2tamper = self.frames[GLOBAL + CROSS] * self.frames[CROSS + TAMPER]
        intermediate = global2tamper * rdk.transl(0, 0, -100) * self.frames[TAMPER + FILTER] * \
            self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]
        start = global2tamper * self.frames[TAMPER + FILTER] * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]

        end = global2tamper * self.frames[SCRAPER + FILTER] * rdk.transl(depth, 0, 0) * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]

        # self.MoveJ(intermediate, "Move to tamper") # Used to determine joint angles
        self.MoveJ(self.joint_angles[TAMPER + ENTRY], "Move to tamper")
        self.MoveJ(start, "Tamper start")
        self.MoveL(end, "Compress coffee")
        self.MoveL(start, "Lower filter")
        self.MoveJ(intermediate, "Remove from tamper")

    def insert_filter_silvia(self):
        """ Move portafilter tool filled with grinds to coffee machine for TA to insert into coffee machine """
        self.log("\n" + STRIP * "-" + " Move filter to silvia " + "-" * STRIP)

        # Calculate position of filter next to coffee machine
        intermediate = rdk.rotz(-1.5) * self.frames[GLOBAL + CROSS] * self.frames[CROSS + TAMPER] * \
            rdk.transl(70, 0, -100) * self.frames[TAMPER + FILTER] * self.frames[FILTER + TOOL] \
            * self.frames[TOOL + TCP]
        entry = self.frames[GLOBAL+SILVIA] * rdk.transl(0, -100, -170) * self.frames[SILVIA+FILTER] \
            * self.frames[FILTER + TOOL] * self.frames[TOOL + TCP]

        # Move filter to coffee machine and allow time for TA to insert into machine
        self.MoveJ(intermediate)
        self.MoveL(entry, "Filter to silvia")
        rdk.pause(15)   # Time for TA to remove filter tool and insert into machine

        # Used to calculate joint angles
        # avoid_silvia = rdk.rotz(HALFPI) * self.frames[GLOBAL + CUPMOUNT]
        # self.MoveJ(avoid_silvia, "Avoid silvia")

        # Remove tool from coffee machine and move to tool mount in preparation for next step
        avoid_silvia_joints = [-88.986265, -77.001952, -78.888716, -114.109332, 90.000000, -178.986265]
        self.MoveJ(avoid_silvia_joints, "Avoid silvia")
        self.MoveJ(self.joint_angles[CUPMOUNT])

    def cup_from_stack(self):
        """ Function that picks up a cup from the cup from the stack
        """
        self.log("\n" + STRIP * "-" + " Get cup from stack " + "-" * STRIP)
        # Attach cup tool
        self.tool_mount(CUP, True)
        # Position at the centre of the top cup
        cup_pickup_matrix = self.frames[GLOBAL + CUPSTACK] * self.frames[CUPSTACK + CUP] \
            * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Used to avoid other tools when coming from the tool mount
        rotate_90 = rdk.rotz(HALFPI) * self.frames[GLOBAL+CUPMOUNT]
        # Puts the robot in the correct orientation to pick up a cup
        intermediate = rdk.transl(0, 0, 150) * self.frames[GLOBAL + CUPSTACK] * self.frames[CUPSTACK + CUP] \
            * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Move operations
        self.MoveJ(rotate_90, "Move to stack")  # Avoid other tools
        self.MoveJ(self.joint_angles[CUPSTACK + ENTRY], "Move to correct orientation")  # get correct orientation
        self.MoveJ(intermediate)
        # Point to move the cup tool up to the top cup
        before_cup = rdk.transl(0, 100, 0) * cup_pickup_matrix
        # Move operation
        self.MoveL(before_cup, "Move to cup level")
        # Open cup tool
        self.cup_tool(OPEN)
        # Linear move to grab a cup!
        self.MoveL(cup_pickup_matrix, "Slide to cup edge")
        # Close cup tool
        self.cup_tool(CLOSE)
        # Point that moves the cup away from the stack
        remove_cup = rdk.transl(0, 0, 300) * cup_pickup_matrix
        # Move operations
        self.MoveL(remove_cup, "Remove cup")
        self.MoveJ(self.joint_angles["rotated" + CUP], "Rotate cup")

    def place_cup(self, height):
        """ Function to place the coffee cup under the coffee CoffeeMachine
        height: varibale used to adjust the height of the cup tool above tamp
        drip tray of the coffee machine
        """
        self.log('\n' + STRIP * '-' + ' Cup to Silvia ' + '-' * STRIP)
        # Point that defines pick up location of the cup
        end_point = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 7, 0) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Stand off position to allow for the cup tool to be opened.
        inter = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 15, -100) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Position further out from the stand off position to ensure that that
        # the tool does not hit the porta filter.
        out = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 50, -150) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]

        # Move operations
        self.MoveJ(out, "Move inline with filter")
        self.MoveJ(inter, "Intermediate point")
        self.MoveJ(end_point, "Move cup to underneath filter")
        # Open cup tool
        self.cup_tool(OPEN)
        # Move operations
        self.MoveJ(inter, "Intermediate point")
        self.cup_tool(CLOSE)
        self.MoveJ(out, "Remove tool from machine")
        # Close cup tool
        self.tool_mount(CUP, False)

    def turn_on_silvia(self, time):
        """ Function used to turn the coffee machine on and off to dispense
        coffee into the cup
        time: the time that the coffee machine will be turned on for
        """
        self.log("\n" + STRIP * "-" + " Turn on coffee machine " + "-" * STRIP)
        # Attach the grinder tool
        self.tool_mount(GRINDER, True)

        # Move to button
        # Position for the on button
        on = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + SILVIAPOWERON] \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        # Position for the off button
        off = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + SILVIAPOWEROFF] \
            * self.frames[PUSHER + TOOL] * self.frames[TOOL + TCP]
        # Intermidiate point that was used to avoid the other tools on the way to the coffee machin
        intermediate_point = rdk.transl(120, 100, 0) * on
        # Z translations to push the buttons
        pushOn = on * rdk.transl(0, 0, 6)
        pushOff = off * rdk.transl(0, 0, 6)
        # Move operations
        self.MoveJ(intermediate_point, "Avoid tools")
        self.MoveJ(on, "Move to button")
        self.MoveL(pushOn, "Push button")
        self.MoveJ(on, "Release")
        # Pause for set time to allow coffee to be made
        rdk.pause(time)
        # Move operations
        self.MoveJ(off, "Move to off")
        self.MoveL(pushOff, "Push button")
        self.MoveJ(off, "Release")
        self.MoveJ(intermediate_point, "Avoid tools")
        # Detach the grinder tool
        self.tool_mount(GRINDER, False)

    def pickup_coffee(self, height):
        """ Function to pick up the coffee cup after it has been filled with CoffeeMachine
        height: adjusts the height of the cup tool from the surface of the drip tray
        """

        self.log('\n' + STRIP * '-' + ' Cup to Rodney ' + '-' * STRIP)
        # Attach the cup tool
        self.tool_mount(CUP, True)

        # Define the end position that picks up the coffee cup
        end_point = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 7, 0) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Stand off point next to the coffee machine.
        inter = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 15, -100) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Position further away from the coffee machine to avoid hitting the portafilter in simulation.
        out = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP] \
            * rdk.transl(height, 60, -180) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Move operations
        self.MoveJ(out, "Cup entry point")
        self.MoveJ(inter, "Intermediate point")
        # Open Cup tool
        self.cup_tool(OPEN)
        self.MoveJ(end_point, "Move to cup")
        # Close cup tool
        self.cup_tool(CLOSE)
        self.MoveJ(inter, "Intermediate point")
        self.MoveJ(out, "Remove cup")
        # Position to move the cup up to get above the coffee machine
        up = rdk.transl(0, 0, 350) * out
        # Position to move the cup over to the centre of the coffee machine
        over_silvia = rdk.transl(0, 0, 50) * down
        # Position to lower the cup down onto the coffee machine
        down = self.frames[GLOBAL + SILVIA] * self.frames[SILVIA + CUP+"place"] \
            * rdk.transl(height-10, 0, 0) * self.frames[CUP + TOOL] * self.frames[TOOL + TCP]
        # Move operations
        self.MoveL(up, "Lift cup up")
        self.MoveJ(over_silvia, "Position over silvia")
        self.MoveJ(down, "Place cup down")
        # Open cup tool
        self.cup_tool(OPEN)
        # Finished!


def main():
    # Read transformation matrices and joint angles from CSV file
    frame_filename = "reference_frames.csv"
    joint_filename = "joint_angles.csv"
    frames = read_frames(frame_filename)
    joint_angles = read_joint_angles(joint_filename)
    # Specify logfile name
    logfile = "output.txt"

    # Initialise robot programming environment and define reference frames
    RDK = rl.Robolink()
    robot = RDK.Item("UR5")
    world_frame = RDK.Item("UR5 Base")
    home = RDK.Item("Home")  # existing target in station
    master_tool = RDK.Item("Master Tool")
    # Set reference frames for robot
    robot.setPoseFrame(world_frame)
    robot.setPoseTool(master_tool)

    # Initialise coffee machine and calculate inverses of required transform matrices
    machine = CoffeeMachine(robot, master_tool, RDK, frames, joint_angles, logfile)
    machine.frames[HOME] = home
    machine.frames[TOOL + TCP] = frames[TCP + TOOL].inv()
    machine.frames[BALL + TOOL] = machine.frames[TOOL + BALL].inv()
    machine.frames[FILTER + TOOL] = machine.frames[TOOL + FILTER].inv()
    machine.frames[PUSHER + TOOL] = machine.frames[TOOL + PUSHER].inv()
    machine.frames[PULLER + TOOL] = machine.frames[TOOL + PULLER].inv()
    machine.frames[CUP + TOOL] = machine.frames[TOOL + CUP].inv()

    # Define constants for operation
    N = 3  # amount of times leaver needs to be pulled
    height = 98  # cup height above base of coffee machine
    time = 12    # Duration to press coffee machine button for
    scraper_height = 8  # distance from scraper to coffee filter
    tamp_height = 15    # depth to push filter into tamper

    # Run coffee making tasks
    machine.insert_filter_grinder()
    machine.turn_on_grinder()
    machine.pull_lever_multiple(N)
    machine.scrape_filter(scraper_height)
    machine.tamp_filter(tamp_height)
    machine.insert_filter_silvia()
    machine.cup_from_stack()
    machine.place_cup(height)
    machine.turn_on_silvia(time)
    machine.pickup_coffee(height)


main()
