from xbot2_interface import pyxbot2_interface as xbi
import pyopensot as pysot
import numpy as np
import time
from pyopensot.tasks.velocity import Cartesian
from pyopensot.constraints.velocity import JointLimits, VelocityLimits, ConvexHull, CartesianPositionConstraint
import array
from scipy.spatial.transform import Rotation as R
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
model.setJointPosition(q)
model.update()

rviz = replay.rvizer(urdf_path)

# w_T_b = TransformStamped()
# w_T_b.header.frame_id = "world"
# w_T_b.child_frame_id = "body"

dt = 1./100.

contact_frames = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

contact_tasks = {}
for contact_frame in contact_frames:
    contact_tasks[contact_frame] = Cartesian(contact_frame, model, contact_frame, "world")

base_task = Cartesian("base", model, "base_link", "world")

# CONSTRAINTS
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
#
dqmax = model.getVelocityLimits()
dqlims = VelocityLimits(model, dqmax, dt)

convex_hull = ConvexHull(model, contact_frames)

# # Planes to constraint the base movement on z
# A = np.array([[0., 0., 1.], [0., 0., -1.]])
# b = np.array([q[2] + 0.01, -q[2] + 0.05])
# base_pos_limits = CartesianPositionConstraint(base_task, A, b)

# STACK
stack = ((contact_tasks[contact_frames[0]][0:3] + contact_tasks[contact_frames[1]][0:3] + contact_tasks[contact_frames[2]][0:3] + contact_tasks[contact_frames[3]][0:3])/base_task) << qlims << dqlims << convex_hull# << base_pos_limits
stack.update()

# SOLVER
solver = pysot.iHQP(stack, eps_regularisation=1e9)

# Markers
lock = threading.Lock()
replay.interactive_marker(rviz.server, base_task, lock)

com_marker = rviz.server.scene.add_icosphere(
    "/com",
    radius=0.02,
    position=(0.5, 0.0, 0.3),
    color=(255, 0, 0),
)

convex_hull_marker = replay.convex_hull_marker(rviz.server)

try:
    while True:
        # Update actual position in the model
        model.setJointPosition(q)
        model.update()

        # Update Stack
        stack.update()
        convex_hull.update() # we update it here because if removed from the stack it will not update

        # Solve
        dq = solver.solve()
        q = model.sum(q, dq)  # we use the model sum to account for the floating-base and wheels manifolds

        com = model.getCOM()
        success, ch = convex_hull.getConvexHull()
        if success:
                for i in range(len(ch)): #express ch in world frame
                        ch[i] += np.array([com[0], com[1], com[2]])
        else:
                print("Convex Hull computation failed.")


        with rviz.server.atomic():
                rviz.viser_urdf.update_cfg(np.append(0., q[7:])) # this is needed because the model contains the floating joint which is seen by viser as an actuated dof
                rviz.base.position = q[:3]
                rviz.base.wxyz = np.array([q[6], q[3], q[4], q[5]])
                rviz.update()
                com_marker.position = np.array([com[0], com[1], 0.])
                convex_hull_marker.update(ch)
        rviz.server.flush()



#         node.publish_planes(A, b, "world")


#         #
        time.sleep(dt)

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")

