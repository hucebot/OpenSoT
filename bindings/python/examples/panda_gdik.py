
from xbot2_interface import pyxbot2_interface as xbi
import time
from pyopensot.tasks.velocity import Cartesian
import numpy as np
import pyopensot as pysot
import replay
import threading


resource = "panda.urdf";
urdf_path = pysot.find(resource)
print(f"Loading {urdf_path}")

urdf_string = pysot.ReadFile(urdf_path)

# Initiliaze node and wait for robot_description parameter
model = xbi.ModelInterface2(urdf_string)

# Set a homing configuration
q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()


# Create a Cartesian task at frame panda_link7, set lambda gain
c = Cartesian("Cartesian", model, "fp3_link8", "world")
c.setLambda(1.)


# Get the actual reference for postural and Cartesian task
pose_ref, vel_ref = c.getReference()


# IK loop
dt = 1./1000.



# Visualization
rviz = replay.rvizer(urdf_path)
q_plot = replay.joint_plot(title="Joint Positions", size=model.nq, legend_label="q", server=rviz.server, dt=dt)

# Markers
lock = threading.Lock()
replay.interactive_marker(rviz.server, c, lock)


try:
    while True:
        # Update actual position in the model
        model.setJointPosition(q)
        model.update()

        c.update()

        J = c.getA()
        e = c.getb()
        q += c.getLambda() * np.matmul(J.transpose(), e)

        rviz.viser_urdf.update_cfg(q)

        q_plot.update(q)

        time.sleep(dt)
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
