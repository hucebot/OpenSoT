import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopensot import AffineHelper, OptvarHelper, GenericTask, AggregatedTask, Task
import pyopensot as pysot
from pyopensot.oc import *
from rclpy.node import Node
from pyopensot.tasks.acceleration import Cartesian, CoM, Postural, AngularMomentum

import rclpy
from ament_index_python.packages import get_package_share_directory
import pathlib

from xbot2_interface import pyxbot2_interface as xbi
import subprocess
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
from tf2_ros import TransformBroadcaster
from ttictoc import tic, toc
import time
from utils import *

class ros2_node(Node):
    def __init__(self):
        super().__init__('go2')
        self.get_logger().info("go2 node has been started.")
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.base_link_broadcaster = TransformBroadcaster(self)
        self.joint_msg = JointState()
        self.w_T_b = TransformStamped()
        self.w_T_b.header.frame_id = "world"
        self.w_T_b.child_frame_id = "base"

    def publish(self, q):
        t = self.get_clock().now().to_msg()

        self.joint_msg.position = q[7::]
        self.joint_msg.header.stamp = t

        self.w_T_b.header.stamp = t
        self.w_T_b.transform.translation.x = q[0]
        self.w_T_b.transform.translation.y = q[1]
        self.w_T_b.transform.translation.z = q[2]
        self.w_T_b.transform.rotation.x = q[3]
        self.w_T_b.transform.rotation.y = q[4]
        self.w_T_b.transform.rotation.z = q[5]
        self.w_T_b.transform.rotation.w = q[6]

        self.joint_state_publisher.publish(self.joint_msg)
        self.base_link_broadcaster.sendTransform(self.w_T_b)

roslaunch = subprocess.Popen(['ros2', 'launch', 'hurobots', 'go2_state_publisher.launch.py'], stdout=subprocess.PIPE, shell=False)

urdf_string = pathlib.Path(get_package_share_directory('hurobots') + "/description_files/urdf/go2/go2.urdf").read_text()

model = xbi.ModelInterface2(urdf_string)

# q_val = [ 0., 0., 0., 0., 0., 0., 1., # base
#        -0.1, 0.,  0., #hips
#         0.432, #knee
#         -0.317, 0., # ankles
#         -0.1, 0.,  0., #hips
#         0.432, #knee
#         -0.317, 0., #ankles
#         0., 0., 0., # waist
#         0.3,  0.25, 0., 1.,  0.15,  0., 0., # arm
#         0.3, -0.25,  0., 1., 0.15,  0.,  0.] # arm

q_init = [
    0.005,
    0.72,
    -1.4,
    -0.005,
    0.72,
    -1.4,
    -0.005,
    0.72,
    -1.4,
    0.005,
    0.72,
    -1.4,
]

q_val = np.concatenate((np.array([0.,0.,0.3262,0.,0.,0.,1.]),q_init))
qdot_val = np.zeros(model.nv)
qddot_val = np.zeros(model.nv)


model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
model.update()


contact_frames = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]


rclpy.init()
ros2node = ros2_node()
ros2node.joint_msg.name = model.getJointNames()[1::]
ros2node.publish(q_val)
time.sleep(0.5)

rclpy.spin_once(ros2node, timeout_sec=2.)


vars = list()
# x
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
vars.append(("qddot", model.nv))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")


dvars = list()
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))
dvars.append(("dqddot", model.nv))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")
dqddot = dvariables.getVariable("dqddot")

x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

dx = AffineHelper.pile(dq, dqdot)
dxdot = AffineHelper.pile(dqdot, dqddot)


Ns = 60 # number of nodes
tf = 2. # final time
dt = tf/Ns 
print(f"Ns: {Ns}, tf: {tf}, dt: {dt}")

x0 = list()
for i in range(Ns+1):
    x0.append(np.concatenate((q_val, qdot_val)))

u0 = list()
for i in range(Ns):
    u0.append(qddot_val)

ocp = pysot.oc.OCP()
dd = list()
const = list()
minus = list()
for i in range(Ns+1):
    stage = Stage()
    """ First we include information related to the state space """
    stage.state_space = CompositeSpace([SE3Space(), VectorSpace(model.nq-7), VectorSpace(model.nv)])

    """ We include both state variables and dvariables """
    stage.x = x
    stage.xdot = xdot
    stage.dx = dx

    if i<Ns:
        """ We include both control variables and dvariables """
        stage.u = qddot
        stage.du = dqddot

    """ We include q and qdot defined for the state variables """
    stage.q = q
    stage.v = qdot
    stage.a = qddot


    stage.model = xbi.ModelInterface2(urdf_string)

    ocp.addStage(stage)

ocp.update(x0, u0)


for i in range(Ns):
    dbase = pysot.oc.EulerSE3(ocp.stage(i).model, dx[:6], dxdot[:6], ocp.stage(i).x[:7], ocp.stage(i).xdot[:6], ocp.stage(i+1).x[:7], dt)
    dpos = pysot.oc.EulerVector(ocp.stage(i).model, dx[6:model.nv], dxdot[6:model.nv], ocp.stage(i).x[7:model.nq], ocp.stage(i).xdot[6:model.nv], ocp.stage(i+1).x[7:model.nq], dt)
    dvel = pysot.oc.EulerVector(ocp.stage(i).model, dx[model.nv:], dxdot[model.nv:], ocp.stage(i).v, ocp.stage(i).a, ocp.stage(i+1).v, dt)
    dd.append(dbase)
    dd.append(dpos)
    dd.append(dvel)
    ocp.stage(i).dynamics_derivative = dbase + dpos + dvel



ocp.update(x0, u0)

alpha = 0.05

costs = []
stack = None
for i in range(Ns):

    minu = min_var.create(f"minu{i}", ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e-9 * np.eye(model.nv))
    costs.append(minu)
    

    mintau = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    mintau.setWeight(0 * np.eye(model.nv))
    costs.append(mintau)
    stack = minu + mintau
    # ocp.stage(i).stack = pysot.AutoStack(minu + mintau)


    # com = CoM(ocp.stage(i).model, dvariables.getVariable("dqddot"))
    # com.setLambda(1.)
    # com_ref, vel_ref, acc_ref = com.getReference()
    # com0 = com_ref.copy()

    cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(Ns).model, dvariables.getVariable("dq"), "base")
    cartesian_task.setWeight(1e0 * np.eye(6))
    costs.append(cartesian_task)
    base_ref = cartesian_task.getReference()



    # base_ref.translation[0] = com0[0] + alpha * np.sin(np.pi * i*dt)
    # base_ref.translation[1] = base_ref.translation[1] + alpha * np.cos(np.pi * i*dt)
    base_ref.translation[2] = base_ref.translation[2] + alpha * np.sin(np.pi * i*dt)

    cartesian_task.setReference(base_ref)
    costs.append(cartesian_task)
    stack += cartesian_task

    # base = Cartesian("base", model, "world", "base", variables.getVariable("qddot"))
    # base.setLambda(1.)


    for frame in contact_frames:
        cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(Ns).model, dvariables.getVariable("dq"), frame)
        cartesian_task.setWeight(1e0 * np.eye(6))
        costs.append(cartesian_task)
        stack += cartesian_task%[0, 1, 2]

    ocp.stage(i).stack = pysot.AutoStack(stack)

    # tau_min
    tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    tau_lims = tau_lim.getTorqueLimit()
    tau_lims[:6] = [1e-9]*6
    tau_lim.setTorqueLimit(tau_lims)
    const.append(tau_lim)
    # ocp.stage(i).stack << tau_lim

minvel = min_var.create(f"minvel", ocp.stage(Ns).x[model.nq:], dvariables.getVariable("dqdot"))
minvel.setWeight(1e0*0 * np.eye(model.nv))

# postural = Postural(ocp.stage(Ns).model)
# postural.setWeight(1e3 * np.eye(model.nv))
# postural.setReference([np.pi, 0.])
ocp.stage(Ns).stack = pysot.AutoStack(minvel)

ocp.update(x0, u0)

print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 10
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1e-2
solver.getOptions().min_abs_delta_solution = 1e-3
solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")



ocp.update(x0, u0)
success = solver.solve(x0, u0)

x0 = solver.getStateSolution()
u0 = solver.getControlSolution()


try:
    t= 0.
    while rclpy.ok():
        input()

        x = x0[0]
        for i in range(len(x0)):
            x = x0[i]
            q_val = x.tolist()[:model.nq]
            # if i<Ns: print(-mintaus[i].getb())
            ros2node.publish(q_val)
            time.sleep(dt)

        ros2node.publish(q_val)

        rclpy.spin_once(ros2node, timeout_sec=0.0)

        # time.sleep(0.001)
        

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    # rviz.kill()
    roslaunch.kill()
    ros2node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()



