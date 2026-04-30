from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import Affine3

from pyopensot.tasks.acceleration import Cartesian, CoM, DynamicFeasibility
from pyopensot.constraints.acceleration import JointLimits, VelocityLimits
from pyopensot.constraints.force import FrictionCone
from pyopensot.variables import Torque
from pyopensot.tasks import MinimizeVariable
import pyopensot as pysot
import numpy as np
import time
import replay
import threading


resource = "go2.urdf";
urdf_path = pysot.find(resource)
print(f"Loading {urdf_path}")

urdf_string = pysot.ReadFile(urdf_path)

model = xbi.ModelInterface2(urdf_string)
qmin, qmax = model.getJointLimits()
dqmax = model.getVelocityLimits()

q = np.array([0., 0., 0.32, 0., 0., 0., 1.,
        0.1, 0.8, -1.5,
        -0.1, 0.8, -1.5,
        0.1, 1.0, -1.5,
        -0.1, 1.0, -1.5])

dq = np.zeros(model.nv)
model.setJointPosition(q)
model.setJointVelocity(dq)
model.update()

rviz = replay.rvizer(urdf_path)
dt = 1./1000.

# Instantiate Variables: qddot and contact forces (3 per contact)
contact_frames = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
variables_vec = dict()
variables_vec["qddot"] = model.nv
for contact_frame in contact_frames:
    variables_vec[contact_frame] = 3
variables = pysot.OptvarHelper(variables_vec)

# Creates tasks cand constraints
com = CoM(model, variables.getVariable("qddot"))
com_ref, vel_ref, acc_ref = com.getReference()
com0 = com_ref.copy()

base = Cartesian("base", model, "base_link", "world", variables.getVariable("qddot"))

contact_tasks = list()
for contact_frame in contact_frames:
    contact_tasks.append(Cartesian(contact_frame + "_kin", model, contact_frame, "world", variables.getVariable("qddot")))

stack = com + (base%[3, 4, 5])
force_variables = list()
for i in range(len(contact_frames)):
    stack = stack + 10.*(contact_tasks[i]%[0, 1, 2])
    force_variables.append(variables.getVariable(contact_frames[i]))

torques = Torque(model=model, qddot_var=variables.getVariable("qddot"), contact_links=contact_frames, force_vars=force_variables)
stack = stack + 1e-6 * MinimizeVariable("min_torques", torques)

# Creates the stack.
# Notice:  we do not need to keep track of the DynamicFeasibility constraint so it is created when added into the stack.
# The same can be done with other constraints such as Joint Limits and Velocity Limits
stack = pysot.AutoStack(stack) << DynamicFeasibility("floating_base_dynamics", model, variables.getVariable("qddot"), force_variables, contact_frames)
stack = stack << JointLimits(model, variables.getVariable("qddot"), qmax, qmin, 10.*dqmax, dt)
stack = stack << VelocityLimits(model, variables.getVariable("qddot"), dqmax, dt)
for i in range(len(contact_frames)):
    T = Affine3()
    mu = (T.linear, 0.8) # rotation is world to contact
    stack = stack << FrictionCone(contact_frames[i], variables.getVariable(contact_frames[i]), model, mu)

# Creates the solver
solver = pysot.iHQP(stack)

lock = threading.Lock()
replay.interactive_marker(rviz.server, base, lock, slider_max=1000, slider_step=10)
replay.com_marker(rviz.server, com, lock, slider_max=1000, slider_step=10)

q_plot = replay.plot(title="Configuration Positions", size=model.nq, legend_label="q", server=rviz.server, dt=dt)
v_plot = replay.plot(title="Configuration Velocities", size=model.nv, legend_label="v", server=rviz.server, dt=dt)
a_plot = replay.plot(title="Configuration Accelerations", size=model.nv, legend_label="v", server=rviz.server, dt=dt)

#
contact_forces_dict = {}
for contact_frame in contact_frames:
    contact_forces_dict[contact_frame] = np.array([0., 0., 0.])
try:
    while True:
        # Update actual position in the model
        model.setJointPosition(q)
        model.setJointVelocity(dq)
        model.update()

        # Variable Update
        torques.update()
#

        # Update Stack
        stack.update()

        # Solve
        x = solver.solve()
        ddq = variables.getVariable("qddot").getValue(x) # from variables vector we retrieve the joint accelerations
        q = model.sum(q, dq*dt + 0.5 * ddq * dt * dt) # we use the model sum to account for the floating-base
        dq += ddq*dt
#
        for contact_frame in contact_frames:
            contact_forces_dict[contact_frame] = variables.getVariable(contact_frame).getValue(x)


        rviz.update(q=np.append(0., q[7:]), base=q[:7], contact_forces_dict=contact_forces_dict)
        q_plot.update(q)
        v_plot.update(dq)
        a_plot.update(ddq)

#       # Sleep to maintain the desired rate
        time.sleep(dt)
#
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")