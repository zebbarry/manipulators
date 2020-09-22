#Jack's testing for the coffee grinder button

# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import robolink as rl  # RoboDK API
import robodk as rdk  # Robot toolbox
# import numpy as np
from reference_frames import *

COMMAND_WAIT = 2

def move_filter(frames):
    pass


# Read transformation matrices from file
filename = 'reference_frames.csv'
frames = read_frames(filename)

# Read frames from station
RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
home = RDK.Item('Home')  # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

# Read subprograms
grinder_tool_attach = RDK.Item('Grinder Tool Attach')
grinder_tool_detach = RDK.Item('Grinder Tool Detach')
filter_tool_attach = RDK.Item('Portafilter Tool Attach')
filter_tool_detach = RDK.Item('Portafilter Tool Detach')
cup_tool_attach = RDK.Item('Cup Tool Attach')
cup_tool_detach = RDK.Item('Cup Tool Detach')

# Pickup filter
robot.MoveJ(frames[GLOBAL+GRINDERMOUNT])
grinder_tool_attach.RunCode(grinder_tool_attach)
rdk.pause(COMMAND_WAIT)
robot.MoveJ(home)
rdk.pause(COMMAND_WAIT)

# Move to ball
#frames[BALL+TOOL] = frames[TOOL+BALL].inv()
frames[TOOL+TCP] = frames[TCP+TOOL].inv()
#global2end = frames[GLOBAL+GRINDER] * frames[GRINDER+BALL] * frames[BALL+TOOL] #* frames[TOOL+TCP]
global2end = frames[GLOBAL+SILVIA] * frames[SILVIA+SILVIAPOWERBUTTON] * frames[TOOL+TCP]
robot.MoveJ(global2end)

robot.MoveJ(frames[GLOBAL+GRINDERMOUNT])
grinder_tool_detach.RunCode(grinder_tool_detach)
