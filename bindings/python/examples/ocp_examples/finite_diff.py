import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopensot import AffineHelper, OptvarHelper, GenericTask, AggregatedTask, Task, VariableXd
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


np.set_printoptions(linewidth=2000, threshold=100000, suppress=True, precision=2)


urdf_string = pathlib.Path(get_package_share_directory('hurobots') + "/description_files/urdf/go2/go2.urdf").read_text()

model = xbi.ModelInterface2(urdf_string)

q_init = [
    0.,
    0.72,
    -1.4,
    -0.,
    0.72,
    -1.4,
    -0.,
    0.72,
    -1.4,
    0.,
    0.72,
    -1.4,
]

q_val = np.concatenate((np.array([0.,0.,0.3258,0.,0.,0.,1.]),q_init))
qdot_val = np.zeros(model.nv)
qddot_val = np.zeros(model.nv)


model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
model.update()


contact_frames = ["RL_foot","FL_foot","RR_foot","FR_foot"]


vars = list()
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
vars.append(("qddot", model.nv))
for frame in contact_frames:
    vars.append((frame+"_force", 3))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")


dvars = list()
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))
dvars.append(("dqddot", model.nv))
for frame in contact_frames:
    dvars.append((frame+"_dforce", 3))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")
dqddot = dvariables.getVariable("dqddot")

x = VariableXd.pile(q, qdot)
xdot = VariableXd.pile(qdot, qddot)

dx = VariableXd.pile(dq, dqdot)
dxdot = VariableXd.pile(dqdot, dqddot)

contact_frames_vars = {}
contact_frames_dvars = {}
for frame in contact_frames:
    contact_frames_vars[frame] = variables.getVariable(frame+"_force")
    contact_frames_dvars[frame] = dvariables.getVariable(frame+"_dforce")

_u = qddot
for frame in contact_frames:
    _u = VariableXd.pile(_u, contact_frames_vars[frame])

_du = dqddot
for frame in contact_frames:
    _du = VariableXd.pile(_du, contact_frames_dvars[frame])

Ns = 5 # number of nodes
tf = 2. # final time
dt = tf/Ns 
print(f"Ns: {Ns}, tf: {tf}, dt: {dt}")



f0 = np.array([0.,0., 0.])#mass*9.81/4.])

x0 = list()
u0 = list()
for i in range(Ns):
    x0.append(np.concatenate((q_val, qdot_val)))
    if i<Ns-1:
        u0.append(qddot_val)
        for frame in contact_frames:
            u0[i] = np.concatenate((u0[i], f0))

ocp = pysot.oc.OCP()
dd = list()
const = list()
minus = list()
for i in range(Ns):
    stage = Stage()
    """ First we include information related to the state space """
    stage.state_space = CompositeSpace([SE3Space(), VectorSpace(model.nq-7), VectorSpace(model.nv)])

    """ We include both state variables and dvariables """
    stage.x = x.copy()
    stage.xdot = xdot.copy()
    stage.dx = dx.copy()

    if i<Ns-1:
        """ We include both control variables and dvariables """
        stage.u =  _u.copy()
        stage.du = _du.copy()

        stage.variables = contact_frames_vars.copy()

    """ We include q and qdot defined for the state variables """
    stage.q = q.copy()
    stage.v = qdot.copy()
    stage.a = qddot.copy()

    stage.model = xbi.ModelInterface2(urdf_string)

    ocp.addStage(stage)

ocp.update(x0, u0)


for i in range(Ns-1):
    dbase = pysot.oc.EulerSE3(ocp.stage(i).model, dx[:6], dxdot[:6], ocp.stage(i).x[:7], ocp.stage(i).xdot[:6], ocp.stage(i+1).x[:7], dt)
    dpos = pysot.oc.EulerVector(ocp.stage(i).model, dx[6:model.nv], dxdot[6:model.nv], ocp.stage(i).x[7:model.nq], ocp.stage(i).xdot[6:model.nv], ocp.stage(i+1).x[7:model.nq], dt)
    dvel = pysot.oc.EulerVector(ocp.stage(i).model, dx[model.nv:], dxdot[model.nv:], ocp.stage(i).v, ocp.stage(i).a, ocp.stage(i+1).v, dt)
    dd.append(dbase)
    dd.append(dpos)
    dd.append(dvel)
    ocp.stage(i).dynamics_derivative = dbase + dpos + dvel



ocp.update(x0, u0)

frame = "RL_foot"

costs = []
stack = None
for i in range(Ns-1):

    # mintau = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    # for frame in contact_frames:
    #     mintau.addForce(frame, contact_frames_vars[frame])
    # mintau.setWeight(1e-3*0 * np.eye(model.nv))
    # costs.append(mintau)
    # stack = mintau


    # cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(i).model, dvariables.getVariable("dq"), "base")
    # cartesian_task.setWeight(1. * np.eye(6))
    # costs.append(cartesian_task)
    # stack = cartesian_task

    contact_task = ContactTask(ocp.stage(i).model, frame, ocp.stage(i).dx, ocp.stage(i).du)
    costs.append(contact_task)
    stack = contact_task


    ocp.stage(i).stack = pysot.AutoStack(stack)



q_base = random_pose(-2.,2.)
# q_base = np.array([0.,0.,0.,0.,0.,0.5,.5])


print(q_base)

q_zero = np.zeros(model.nq + model.nv)
q_zero[6] = 1.

q_val = np.concatenate((q_base,q_init))
qdot_val = np.random.rand(model.nv)
qddot_val = np.zeros(model.nv)

f0 = np.array([0.,0., 0.])

x0 = list()
u0 = list()
for i in range(Ns):
    x0.append(np.concatenate((q_val, qdot_val)))
    if i<Ns-1:
        u0.append(qddot_val)
        for frame in contact_frames:
            u0[i] = np.concatenate((u0[i], f0))



ocp.update(x0, u0)


import unittest
utest = unittest.TestCase()

STAGE = 1
eps   = 1e-6


Jac = costs[STAGE].getA().copy()
val = costs[STAGE].getb()



print(Jac.shape)
print(ocp.stage(STAGE).model.nv)
print(dx.getInputSize())

M = Jac.shape[0]
N = dx.getInputSize()

JacDiff = np.zeros((M, N))
print(JacDiff.shape)

# print(Jac)
print(val)





x_space = CompositeSpace([SE3Space(), VectorSpace(model.nq-7), VectorSpace(model.nv)])
u_space = CompositeSpace([VectorSpace(model.nv + 3*4)])

_x0 = x0.copy()
_u0 = u0.copy()


print("-"*200)

for i in range(N):
    dx = np.zeros(x_space.nv())
    du = np.zeros(u_space.nv())

    if i < dx.size:
        dx[i] += eps
        _x0[STAGE] = x_space.plus(x0[STAGE], dx)
    else:
        du[i- dx.size] += eps
        _u0[STAGE] = u_space.plus(u0[STAGE], du)
    ocp.update(_x0, _u0)
    valp =  - costs[STAGE].getb().copy()

    dx = np.zeros(x_space.nv())
    du = np.zeros(x_space.nv())
    if i < dx.size:
        dx[i] -= eps
        _x0[STAGE] = x_space.plus(x0[STAGE], dx)
    else:
        du[i- dx.size] -= eps
        _u0[STAGE] = u_space.plus(u0[STAGE], du)
    ocp.update(_x0, _u0)
    valm = - costs[STAGE].getb().copy()

    # print(valp)
    # print(valm)
    

    JacDiff[:, i: i+1] = ((valp - valm) / (2.0 * eps)).reshape((-1, 1))


print(Jac)
print("-"*100)
print(JacDiff)

# print(Jac.T[-12:,:])
# print("-"*100)
# print(JacDiff.T[-12:,:])



utest.assertTrue(np.allclose(JacDiff, Jac, 1e-8, 1e-5))



# JacDiff = finite_diff(self.opti.constraints, xx, self.opti.cons_dim)

# def finite_diff(f, z, M, eps=1e-6):
#     N = z.shape[0]
#     jac = np.zeros((M, N))
#     for i in range(N):
#         zp = np.copy(z)
#         zp[i] += eps
#         zm = np.copy(z)
#         zm[i] -= eps
#         jac[:, i : i + 1] = ((f(zp) - f(zm)) / (2.0 * eps)).reshape((-1, 1))
#     return jac




