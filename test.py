# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np
from reference_frames import *

filename = 'reference_frames.csv'

frames = read_frames(filename)

RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


#Existing subprograms
grinder_tool_attach = RDK.Item('Grinder Tool Attach')
grinder_tool_detach = RDK.Item('Grinder Tool Detach')

# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

# Joint angles
J_intermediatepoint = [-151.880896, -97.616411, -59.103383, -112.890980, 90.242082, -161.879346]

# Convert a numpy array into a Mat (e.g.after calculation)
T_grinderapproach_np = np.array([[     0.173648,    -0.984800,    -0.004000,  -502.103741],
    [ -0.984789,    -0.173618,    -0.006928,  -145.353888 ],
    [  0.006128,     0.005142,    -0.999968,   535.250260 ],
    [  0.000000,     0.000000,     0.000000,     1.000000 ]])

T_grinderapproach = rdk.Mat(T_grinderapproach_np.tolist())

robot.MoveJ(T_home, blocking=True)
# robot.MoveJ(J_intermediatepoint, blocking=True)
robot.MoveL(frames[GLOBAL+CUP].transform, blocking=True)

# grinder_tool_attach.RunCode(grinder_tool_attach) # call subprogram
# rdk.pause(3)  # to allow subprogram to complete
# grinder_tool_detach.RunCode(grinder_tool_detach) # call subfunction

# # The following pause is very important - if it is not present, or long enough
# # the frame reset below it occurs before the subprogram completes and this
# # causes problems...
# rdk.pause(3)    
# # Note, the subfunctions change the reference frame, so you need to change it back
# # after calling them
# robot.setPoseFrame(world_frame)
# # you may also need to reset the toolframe
# robot.setPoseTool(robot.PoseTool())

# # and... move home to an existing target
# robot.MoveJ(target)
