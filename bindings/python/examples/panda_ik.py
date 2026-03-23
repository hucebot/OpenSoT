import os
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, MinimumEffort
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
import pyopensot as pysot
import numpy as np
import time
import replay
import threading

resource = "panda.urdf";
urdf_path = pysot.find(resource)
print(f"Loading {urdf_path}")

urdf_string = pysot.ReadFile(urdf_path)

model = xbi.ModelInterface2(urdf_string)
# Set a homing configuration
q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()
#
# Get Joint Limits and Velocity Limits, define dt
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
#
dqmax = model.getVelocityLimits()
dt = 1./100.
dqlims = VelocityLimits(model, dqmax, dt)
#
# Create postural task (it is created at the q configuration previously set
p = Postural(model)
#
# Create a Cartesian task at frame panda_link7, set lambda gain
c = Cartesian("Cartesian", model, "fp3_link8", "world")
c.setLambda(0.1)
#
# Retrieve actual pose of the frame to be used as reference
ref = c.getActualPose().copy()
#
# Create the stack:
# 1st priority Cartesian position
# 2nd priority Cartesian orientation
# 3rd priority postural
s =  (c / p) << qlims
s<<dqlims
s.update()
#
# Get the actual reference for postural and Cartesian task
qref, dqref = p.getReference()
print(f"qref: {qref}")
print(f"dqref: {dqref}")
pose_ref, vel_ref = c.getReference()
print(f"pose_ref: {pose_ref}")
print(f"vel_ref: {vel_ref}")
#
# Creates iHQP solver with stack (using qpOASES as backend)
#
solver = pysot.iHQP(s, eps_regularisation=1e6)

# Visualization
rviz = replay.rvizer(urdf_path)
q_plot = replay.joint_plot(title="Joint Positions", size=model.nq, legend_label="q", server=rviz.server, dt=dt)
v_plot = replay.joint_plot(title="Joint Velocities", size=model.nv, legend_label="v", server=rviz.server, dt=dt)

# Markers
lock = threading.Lock()
replay.interactive_marker(rviz.server, c, lock)

# IK loop
t = 0.
try:
    while True:
        # Update actual position in the model
        model.setJointPosition(q)
        model.update()

        # Update Stack
        s.update()
#
        # Solve
        dq = solver.solve()
        q += dq # Is fixed base so we can simply sum the result
        rviz.viser_urdf.update_cfg(q)

        q_plot.update(q)
        v_plot.update(dq/dt)
#
        time.sleep(dt)
#
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
