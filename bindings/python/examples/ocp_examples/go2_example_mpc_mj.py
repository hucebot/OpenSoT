import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopensot import OptvarHelper, VariableXd
import pyopensot as pysot
from pyopensot.oc import *
from rclpy.node import Node
from pyopensot.constraints.velocity import JointLimits
from pyopensot.tasks.velocity import Postural

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

import mujoco
import mujoco.viewer

np.set_printoptions(linewidth=2000, threshold=100000, suppress=True, precision=10, sign= ' ')


urdf_string = pathlib.Path(get_package_share_directory('huro') + "/resources/description_files/urdf/go2/go2.urdf").read_text()


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

q_weights = np.ones(model.getNv())

w_shoulder = 1e1
# q_weights[:6] = q_weights[:6]*0
q_weights[6]  = w_shoulder * q_weights[6]
q_weights[9]  = w_shoulder * q_weights[9]
q_weights[12] = w_shoulder * q_weights[13]
q_weights[15] = w_shoulder * q_weights[15]

w_elbow = 1e-1
# q_weights[7]  = w_elbow * q_weights[7]
# q_weights[10]  = w_elbow * q_weights[10]
# q_weights[13]  = w_elbow * q_weights[13]
# q_weights[16]  = w_elbow * q_weights[16]

q_weights[8]  = w_elbow * q_weights[8]
q_weights[11]  = w_elbow * q_weights[11]
q_weights[14]  = w_elbow * q_weights[14]
q_weights[17]  = w_elbow * q_weights[17]


q_val = np.concatenate((np.array([0.,0.,0.3258,0.,0.,0.,1.]),q_init))
qdot_val = np.zeros(model.nv)
qddot_val = np.zeros(model.nv)


model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
qmin, qmax = model.getJointLimits()
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



DT = 0.02
Ns = 10

contact_scheduler = Scheduler()
contact_scheduler.addContact("rl", ["RL_foot"])
contact_scheduler.addContact("rr", ["RR_foot"])
contact_scheduler.addContact("fl", ["FL_foot"])
contact_scheduler.addContact("fr", ["FR_foot"])
contact_scheduler.addContact("all", ["FR_foot", "FL_foot", "RR_foot", "RL_foot"])
contact_scheduler.addContact("fr_air", ["FL_foot", "RR_foot", "RL_foot"])
contact_scheduler.addContact("fl_air", ["FR_foot", "RR_foot", "RL_foot"])
contact_scheduler.addContact("rr_air", ["FR_foot", "FL_foot", "RL_foot"])
contact_scheduler.addContact("rl_air", ["FR_foot", "FL_foot", "RR_foot"])


gaits = ["stance", "walk", "jump", "trot"]

contact_scheduler.addPhase(["all"], .2, sequence_name="stance")

contact_scheduler.addPhase(["all"], .3, sequence_name="jump")
contact_scheduler.addPhase(["air"], .2, sequence_name="jump")

contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["fl_air"], duration=.15, sequence_name="walk")
contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["rr_air"], duration=.15, sequence_name="walk")
contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["fr_air"], duration=.15, sequence_name="walk")
contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["rl_air"], duration=.15, sequence_name="walk")

contact_scheduler.addPhase(["all"], .2, sequence_name="trot")
contact_scheduler.addPhase(["rr", "fl"], .15,sequence_name="trot")
contact_scheduler.addPhase(["all"], .2,sequence_name="trot")
contact_scheduler.addPhase(["rl", "fr"], .15, sequence_name="trot")


contact_scheduler.addPhase(["all"], .5, sequence_name="oneleg")
contact_scheduler.addPhase(["rl_air"], .3,sequence_name="oneleg")


gait = "trot"


frame_contact_seq = contact_scheduler.getSequence(DT, nodes_number = Ns,  sequence_name=gait)




_u = qddot
for frame in contact_frames:
    _u = VariableXd.pile(_u, contact_frames_vars[frame])

_du = dqddot
for frame in contact_frames:
    _du = VariableXd.pile(_du, contact_frames_dvars[frame])


mass = model.getMass()
f0 = np.array([0.,0.,0.])

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
    print(i)
    dbase =   pysot.oc.EulerSE3(ocp.stage(i).model, dx[:6], dxdot[:6], ocp.stage(i).x[:7], ocp.stage(i).xdot[:6], ocp.stage(i+1).x[:7], DT)
    dpos = pysot.oc.EulerVector(ocp.stage(i).model, dx[6:model.nv], dxdot[6:model.nv], ocp.stage(i).x[7:model.nq], ocp.stage(i).xdot[6:model.nv], ocp.stage(i+1).x[7:model.nq], DT)
    dvel = pysot.oc.EulerVector(ocp.stage(i).model, dx[model.nv:], dxdot[model.nv:], ocp.stage(i).v, ocp.stage(i).a, ocp.stage(i+1).v, DT)
    dd.append(dbase)
    dd.append(dpos)
    dd.append(dvel)
    ocp.stage(i).dynamics_derivative = dbase + dpos + dvel

ocp.update(x0, u0)

costs = []
calctaus = []
qlims = list()

base_vel = []
cost_list = []
constraints = []
for i in range(Ns):
    stack = None

    # minvel = min_var.create(f"minvel", ocp.stage(i).x[model.nq:], ocp.stage(i).dx[model.nv:])
    minvel = MinVar(f"minvel", ocp.stage(i).dx[model.nv:], ocp.stage(i).x[model.nq:])
    minvel.setWeight(1e-8  *  np.eye(model.nv))
    print(minvel.getWeight())
    if i==Ns-1:
        minvel.setWeight(1e3  *  np.eye(model.nv))
    costs.append(minvel)
    stack = minvel[:model.nv]

    if i < Ns-1:
        # minqddot = min_var.create(f"minqddot{i}", ocp.stage(i).u, ocp.stage(i).du)
        minqddot = MinVar(f"minqddot{i}", ocp.stage(i).du, ocp.stage(i).u)
        minqddot.setWeight(np.eye(model.nv + 4*3))
        costs.append(minqddot)
        stack += 1e-9 * minqddot[0:6]
        stack += 1e-8 * minqddot[6:model.nv]
        stack += 1e-7 * minqddot[model.nv:]


# Base
    cartesian_task = pysot.oc.PosSO3Task("Cartesian", ocp.stage(i).model, ocp.stage(i).dx[:model.nv], "base")
    cartesian_task.setWeight(1e-4 * np.eye(6))
    target = cartesian_task.getReference()
    target.translation[2] = 0.32
    cartesian_task.setReference(target)
    costs.append(cartesian_task)
    stack += cartesian_task[2]

# Base velocity
    cartesian_vel_task = pysot.oc.SE3VelTask("Cartesian", ocp.stage(i).model, ocp.stage(i).dx[:model.nv], "base")
    cartesian_vel_task.setReferenceVelocity([.0,.0,0.,0.,0.,0.])
    cartesian_vel_task.setWeight(1e-3 * np.eye(6))
    base_vel.append(cartesian_vel_task)
    # stack += cartesian_vel_task


    #Feet air
    cost_list.append({})
    cost_list[i]["feet_height"] = {}
    cost_list[i]["feet_vel"] = {}
    for frame in contact_frames:
        cartesian_task = PosSO3Task("Cartesian", ocp.stage(i).model, ocp.stage(i).dx[:model.nv], frame)
        cartesian_task.setWeight(1e0 * np.eye(6))
        cost_list[i]["feet_height"][frame] = cartesian_task
        stack += cartesian_task[2]

        cartesian_vel_task_ = pysot.oc.SE3VelTask("Cartesian", ocp.stage(i).model, dvariables.getVariable("dq"), frame)
        cartesian_vel_task_.setReferenceVelocity([.0,.0,0.,0.,0.,0.])
        cartesian_vel_task_.setWeight(1e-6 * np.eye(6))
        cost_list[i]["feet_vel"][frame] = cartesian_vel_task_
        # stack += cartesian_vel_task_




# Postural
    postural = Postural(ocp.stage(i).model)
    postural.setWeight(1e-3 * np.diag(q_weights))
    minus.append(postural)
    stack +=  AffineTask.toAffine(postural[6:], dvariables.getVariable("dq"))



#Compute Torques
    if i<Ns-1:
        tau_compute = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in frame_contact_seq[i]:
            tau_compute.addForce(frame, contact_frames_vars[frame])
        tau_compute.setWeight(1e-9 * np.eye(ocp.stage(i).model.nv))
        calctaus.append(tau_compute)
        stack += tau_compute

    ocp.stage(i).stack = pysot.AutoStack(stack)    

# Joint Limits
    qlims_i = JointLimits(ocp.stage(i).model, qmax, qmin)
    qlims.append(qlims_i)
    ocp.stage(i).stack = ocp.stage(i).stack << AffineConstraint.toAffine(qlims_i, dvariables.getVariable("dq"))


    constraints.append({})
    constraints[i]["contact"] = {}
    for frame in contact_frames:
        contact_task = ContactConstraint(ocp.stage(i).model, frame, ocp.stage(i).dx)
        constraints[i]["contact"][frame] = contact_task
        if frame in frame_contact_seq[i]:
            contact_task.activate(0.)
        ocp.stage(i).stack = ocp.stage(i).stack << contact_task


    if i < Ns-1:
# Dynamics 
        tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in frame_contact_seq[i]:
            tau_lim.addForce(frame, contact_frames_vars[frame])
        tau_lims = tau_lim.getTorqueLimit()
        tau_lims[:6] = [1e-9]*6
        tau_lim.setTorqueLimit(tau_lims)
        constraints[i]["dynamics"] = tau_lim
        ocp.stage(i).stack = ocp.stage(i).stack << tau_lim

        
        for frame in frame_contact_seq[i]:
            friction_const = FrictionConeConstraint(ocp.stage(i).model, frame, contact_frames_vars[frame], ocp.stage(i).dx, ocp.stage(i).du)
            friction_const.setCoefficient(0.8)
            const.append(friction_const)
            ocp.stage(i).stack = ocp.stage(i).stack << friction_const


ocp.update(x0, u0)

print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = 2
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1e-4
solver.getOptions().min_abs_delta_solution = 1e-2
solver.getOptions().hessian_scale_factor_up = 1e6

# solver.getQPSolver().getOptions().mode = pysot.HpipmMode.Speed
solver.getQPSolver().getOptions().iter_max = 1000
solver.getQPSolver().getOptions().tol_ineq = 1e-2
solver.getQPSolver().getOptions().tol_eq = 1e-2
solver.getQPSolver().getOptions().tol_stat = 1e-3
solver.getQPSolver().getOptions().tol_comp = 1e-3

solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")



ocp.update(x0, u0)
success = solver.solve(x0, u0)


solver.getOptions().optimize_first_state = 1
solver.getOptions().optimize_first_state_cost = 1e3


# solver.getQPSolver().getOptions().mode = pysot.HpipmMode.Speed
solver.getOptions().max_iters = 4
solver.getOptions().verbose = 1
solver.getOptions().line_search_strategy = 2
solver.getQPSolver().getOptions().iter_max = 100
solver.getOptions().wall_time = 0.02
solver.init()

print("inited")


h_feet_target = cost_list[0]["feet_height"][contact_frames[0]].getReference()


xml_path =  "/home/ros2_ws/src/huro/resources/description_files/xml/go2/go2.xml"

mj_model = mujoco.MjModel.from_xml_path(xml_path)
sim_data = mujoco.MjData(mj_model)

# Simulation and control parameters
mj_model.opt.timestep = DT

# PD control gains for tracking MPC trajectory
kp_joints = np.ones(12) * 60  # Position gains for 12 joints (3 per leg)
kd_joints = np.ones(12) * .5     # Velocity gains for 12 joints


print( model.getJointNames()[1:])

try:
    t= 0.

    with mujoco.viewer.launch_passive(mj_model, sim_data) as viewer:

        sim_data.qpos[:] = q_val
        sim_data.qpos[2] +=0.01
        sim_data.qpos[3] =-1
        sim_data.qpos[6] =0

        mujoco.mj_step(mj_model, sim_data)
        if viewer.is_running():
            viewer.sync()

        # input()

        while True:

            # Update MPC state from MuJoCo simulation
            # MuJoCo uses different quaternion convention (w,x,y,z) vs OpenSoT (x,y,z,w)
            q_mj = sim_data.qpos.copy()
            qdot_mj = sim_data.qvel.copy()

            # Convert quaternion from MuJoCo (w,x,y,z) to OpenSoT (x,y,z,w)
            q_opensot = np.zeros(model.nq)
            q_opensot[:3] = q_mj[:3]  # position
            q_opensot[3:6] = q_mj[4:7]  # quaternion xyz
            q_opensot[6] = q_mj[3]  # quaternion w
            q_opensot[7:] = q_mj[7:]  # joint positions

            # Update initial state for MPC with current simulation state
            # x0[0] = np.concatenate((q_opensot, qdot_mj))

            # Update contact sequence based on current time
            frame_contact_seq = contact_scheduler.getSequence(DT, nodes_number = Ns, current_time = t,  sequence_name=gait)

            # Update MPC problem for each stage
            for i in range(Ns):
                # Update base velocity reference
                if t>=1.:
                    base_vel[i].setReferenceVelocity([.2,.0,0.,0.,0.,0.])

                # Update contact constraints and cost functions
                for frame in contact_frames:
                    if frame in frame_contact_seq[i]:
                        # Foot is in contact
                        constraints[i]["contact"][frame].activate(0.)
                        if i<Ns-1:
                            constraints[i]["dynamics"].addForce(frame, contact_frames_vars[frame])
                            calctaus[i].addForce(frame, contact_frames_vars[frame])
                        h_feet_target.translation[2] = 0.
                        cost_list[i]["feet_height"][frame].setReference(h_feet_target)
                    else:
                        # Foot is in the air
                        constraints[i]["contact"][frame].deactivate()
                        if i<Ns-1:
                            constraints[i]["dynamics"].removeForce(frame)
                            calctaus[i].removeForce(frame)
                        h_feet_target.translation[2] = 0.05
                        cost_list[i]["feet_height"][frame].setReference(h_feet_target)

            # Solve MPC optimization (open-loop, no state feedback to MPC)
            suc = solver.solve(x0, u0)
            x0 = solver.getStateSolution()
            u0 = solver.getControlSolution()

            # Get desired joint states from MPC solution (first node)
            q_des_opensot = x0[1][:model.nq]
            qdot_des = x0[1][model.nq:]*DT

            # Extract desired joint positions and velocities (skip base)
            q_des_joints = q_des_opensot[7:]  # Desired joint positions from MPC
            qdot_des_joints = qdot_des[6:]    # Desired joint velocities from MPC

            # Current joint states from MuJoCo (feedback for impedance control)
            q_curr_joints = sim_data.qpos[7:]     # Current joint positions
            qdot_curr_joints = sim_data.qvel[6:]  # Current joint velocities

            # Feedforward torques from MPC dynamics
            tau_ff = calctaus[0].getb()[6:]

            # Joint impedance control: tau = tau_ff + Kp*(q_des - q) + Kd*(qdot_des - qdot)
            tau_impedance = kp_joints * (q_des_joints - q_curr_joints) + kd_joints * (qdot_des_joints - qdot_curr_joints)

            # Combined control: feedforward + impedance feedback
            control_torques = tau_ff + tau_impedance

            # Apply control torques to MuJoCo simulation
            sim_data.ctrl[:] = control_torques
            # sim_data.qpos[7:] = q_des_joints

            # Step MuJoCo simulation forward
            mujoco.mj_step(mj_model, sim_data)

            # Update viewer
            if viewer.is_running():
                viewer.sync()
            else:
                break

            # Shift horizon for warm-starting next iteration
            for i in range(len(x0)-1):
                x0[i] = x0[i+1]
            for i in range(len(u0)-1):
                u0[i] = u0[i+1]

            t += DT
        

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
