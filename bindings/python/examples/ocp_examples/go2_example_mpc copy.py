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

np.set_printoptions(linewidth=2000, threshold=100000, suppress=True, precision=10, sign= ' ')


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

roslaunch = subprocess.Popen(['ros2', 'launch', 'huro', 'go2_rviz.launch.py'], stdout=subprocess.PIPE, shell=False)

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


# print(model.getPose("RL_foot"))
# print(model.getPose("FL_foot"))
# print(model.getPose("RR_foot"))
# print(model.getPose("FR_foot"))
# input()

contact_frames = ["RL_foot","FL_foot","RR_foot","FR_foot"]

rclpy.init()
ros2node = ros2_node()

forcesnode = force_node()
forcesnode.initialize_force_publishers(contact_frames)

ros2node.joint_msg.name = model.getJointNames()[1::]
ros2node.publish(q_val)
# time.sleep(0.5)

rclpy.spin_once(ros2node, timeout_sec=2.)


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
contact_scheduler.addContact("air", [])

# contact_scheduler.addPhase(["all"], .5)
# contact_scheduler.addPhase(["air"], .2)

contact_scheduler.addPhase(["all"], .1)
contact_scheduler.addPhase(["rr", "fl"], .2)
contact_scheduler.addPhase(["all"], .1)
contact_scheduler.addPhase(["rl", "fr"], .2)



frame_contact_seq = contact_scheduler.getSequence(DT, nodes_number = Ns)


# print(f"Ns: {Ns}, tf: {tf}, dt: {DT}")


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

    minvel = min_var.create(f"minvel", ocp.stage(i).x[model.nq:], ocp.stage(i).dx[model.nv:])
    minvel.setWeight(1e-6  *  np.eye(model.nv))
    print(minvel.getWeight())
    if i==Ns-1:
        minvel.setWeight(1e3  *  np.eye(model.nv))
    costs.append(minvel)
    stack = minvel[:model.nv]

    if i < Ns-1:
        minqddot = min_var.create(f"minqddot{i}", ocp.stage(i).u, ocp.stage(i).du)
        minqddot.setWeight(np.eye(model.nv + 4*3))
        costs.append(minqddot)
        stack += 1e-8 * minqddot[0:model.nv]
        stack += 1e-7 * minqddot[model.nv:]


# Base
    cartesian_task = pysot.oc.PosSO3Task("Cartesian", ocp.stage(i).model, ocp.stage(i).dx[:model.nv], "base")
    cartesian_task.setWeight(1e-4 * np.eye(6))
    target = cartesian_task.getReference()
    target.translation[2] = 0.32
    cartesian_task.setReference(target)
    costs.append(cartesian_task)
    stack += cartesian_task[2]#%[3,4,5]

# Base velocity
    cartesian_vel_task = pysot.oc.SE3VelTask("Cartesian", ocp.stage(i).model, ocp.stage(i).dx[:model.nv], "base")
    cartesian_vel_task.setReferenceVelocity([.0,.0,0.,0.,0.,0.])
    cartesian_vel_task.setWeight(1e-3 * np.eye(6))
    base_vel.append(cartesian_vel_task)
    stack += cartesian_vel_task


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
        # stack += tau_compute

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
            friction_const.setCoefficient(0.9)
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

# solver.getQPSolver().getOptions().mode = pysot.HpipmMode.Speed
solver.getOptions().max_iters = 4
solver.getOptions().verbose = 1
solver.getOptions().line_search_strategy = 0
solver.getQPSolver().getOptions().iter_max = 100
solver.getOptions().wall_time = 0.02
solver.init()

print("inited")
input()

force_msgs = {}
for contact_frame in contact_frames:
    force_msgs[contact_frame] = WrenchStamped()
    force_msgs[contact_frame].header.frame_id = contact_frame
    force_msgs[contact_frame].wrench.torque.x = force_msgs[contact_frame].wrench.torque.y = force_msgs[contact_frame].wrench.torque.z = 0.


h_feet_target = cost_list[0]["feet_height"][contact_frames[0]].getReference()
h_feet_target.translation[2] = 0.05


# dt_sim = 0.001
try:
    t= 0.
    while rclpy.ok():
        frame_contact_seq = contact_scheduler.getSequence(DT, nodes_number = Ns, current_time = t)

        for i in range(Ns):
            if t>=1.:
               base_vel[i].setReferenceVelocity([.0,.0,0.,0.,0.,0.5])

            for frame in contact_frames:
                if frame in frame_contact_seq[i]:
                    constraints[i]["contact"][frame].activate(0.)
                    if i<Ns-1: 
                        constraints[i]["dynamics"].addForce(frame, contact_frames_vars[frame])
                        calctaus[i].addForce(frame, contact_frames_vars[frame])
                    h_feet_target.translation[2] = 0.
                    cost_list[i]["feet_height"][frame].setReference(h_feet_target)
                else:
                    constraints[i]["contact"][frame].deactivate()
                    if i<Ns-1: 
                        constraints[i]["dynamics"].removeForce(frame)
                        calctaus[i].removeForce(frame)
                    h_feet_target.translation[2] = 0.05
                    cost_list[i]["feet_height"][frame].setReference(h_feet_target)
        
        
        suc = solver.solve(x0, u0)
        x0 = solver.getStateSolution()
        u0 = solver.getControlSolution()

        q_val = x0[1].tolist()[:model.nq]
        ros2node.publish(q_val)

        j=0
        i=1
        for contact_frame in contact_frames:
            f_local = u0[i][model.nv + j*3: model.nv + j*3+3]

            force_msgs[contact_frame].wrench.force.x = f_local[0]
            force_msgs[contact_frame].wrench.force.y = f_local[1]
            force_msgs[contact_frame].wrench.force.z = f_local[2]
            j+=1

        forcesnode.publish(force_msgs)

        for i in range(len(x0)-1):
            x0[i] = x0[i+1]
        for i in range(len(u0)-1):
            u0[i] = u0[i+1]    
        # u0[-1] = u0[-1]*0.
        
        t += DT
       
        rclpy.spin_once(ros2node, timeout_sec=0.00001)
        

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    # rviz.kill()
    # roslaunch.kill()
    ros2node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()
