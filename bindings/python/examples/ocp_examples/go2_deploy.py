import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopensot import OptvarHelper, VariableXd
import pyopensot as pysot
from pyopensot.oc import *
from rclpy.node import Node
from pyopensot.constraints.velocity import JointLimits
from pyopensot.tasks.velocity import Postural

import rclpy


from xbot2_interface import pyxbot2_interface as xbi
import subprocess
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
from tf2_ros import TransformBroadcaster
from ttictoc import tic, toc
import time
from utils import *

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

np.set_printoptions(linewidth=2000, threshold=100000, suppress=True, precision=1)



class ros2_node(Node):
    def __init__(self):
        name = "go2_deploy_node"
        super().__init__(name)
        self.get_logger().info(f"{name} node has been started.")

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        self.robot_description_subscriber = self.create_subscription(
            String,
            'robot_description',
            self.listener_callback,
            qos_profile)


        # self.joint_states_subsriber = self.create_subscription(
        #     JointState,             # message type
        #     '/joint_states',      # topic name
        #     self.joint_states_callback,      # callback function
        #     10                       # QoS (queue size)
        # )
        # self.get_logger().info('JointSubscriber node has been started.')


        self.urdf=None
        self.state = None
        while self.urdf is None:
            rclpy.spin_once(self)

        self.get_logger().info(f"{name} initialization complete")


    # def joint_states_callback(self, msg: JointState):
    #     self.state = np.concatenate((msg.position , msg.velocity))

    def listener_callback(self, msg):
        self.get_logger().info("URDF readed")
        self.urdf = msg.data

    def publish(self, joint_state_msg:JointState):
        self.joint_state_publisher.publish(joint_state_msg)

rclpy.init()
ros2node = ros2_node()
urdf_string = ros2node.urdf
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

q_weights = np.ones(model.nv)

w_shoulder = 1e3
q_weights[:6] = q_weights[:6]*0
# q_weights[6]  = w_shoulder * q_weights[6]
# q_weights[9]  = w_shoulder * q_weights[9]
# q_weights[12] = w_shoulder * q_weights[13]
# q_weights[15] = w_shoulder * q_weights[15]

# w_elbow = 1e-3
# q_weights[8]  = w_elbow * q_weights[8]
# q_weights[11]  = w_elbow * q_weights[11]
# q_weights[14]  = w_elbow * q_weights[14]
# q_weights[17]  = w_elbow * q_weights[17]


q_val = np.concatenate((np.array([0.,0.,0.3258,0.,0.,0.,1.]),q_init))
qdot_val = np.zeros(model.nv)
qddot_val = np.zeros(model.nv)


model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
model.update()


# print(model.getPose("RL_foot"))
# print(model.getPose("FL_foot"))
# print(model.getPose("RR_foot"))
# print(model.getPose("FR_foot"))
# input()

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



DT = 0.025

# CONTACT SCHEDULING
contacts_dict = {
    "rl_foot": ["RL_foot"],
    "rr_foot": ["RR_foot"],
    "fl_foot": ["FL_foot"],
    "fr_foot": ["FR_foot"],
}

contact_scheduler = ContactScheduler(dt=DT, contact_frame_dict=contacts_dict)

contact_scheduler.add_phase(["rl_foot", "rr_foot", "fr_foot", "fl_foot"], .5)
# contact_scheduler.add_phase(["rl_foot", "rr_foot"], .5)
contact_scheduler.add_phase([], .2)
# contact_scheduler.add_phase(["rl_foot"], 1.)

# for i in range(2):
#     contact_scheduler.add_phase(["rl_foot", "fr_foot"], .2)
#     contact_scheduler.add_phase(["rl_foot", "rr_foot", "fr_foot", "fl_foot"], .2)
#     contact_scheduler.add_phase(["rr_foot", "fl_foot"], .2)
#     contact_scheduler.add_phase(["rl_foot", "rr_foot", "fr_foot", "fl_foot"], .2)
contact_scheduler.add_phase(["rl_foot", "rr_foot", "fr_foot", "fl_foot"], .5)

frame_contact_seq = contact_scheduler.contact_sequence_fnames



Ns = contact_scheduler.total_nodes
tf = Ns * DT
print(f"Ns: {Ns}, tf: {tf}, dt: {DT}")


_u = qddot
for frame in contact_frames:
    _u = VariableXd.pile(_u, contact_frames_vars[frame])

_du = dqddot
for frame in contact_frames:
    _du = VariableXd.pile(_du, contact_frames_dvars[frame])


mass = model.getMass()
f0 = np.array([0.,0.,0.])#mass*9.81/4.])

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
mintaus = []
qlims = list()
for i in range(Ns):
    stack = None

    minvel = min_var.create(f"minvel", ocp.stage(i).x[model.nq:], dvariables.getVariable("dqdot"))
    minvel.setWeight(1e-9  *  np.eye(model.nv))
    if i==Ns-1:
        minvel.setWeight(1e3  *  np.eye(model.nv))
    costs.append(minvel)
    stack = minvel

    if i < Ns-1:
        minqddot = min_var.create(f"minqddot{i}", ocp.stage(i).u, ocp.stage(i).du)
        minqddot.setWeight(np.eye(model.nv + 4*3))
        costs.append(minqddot)
        stack += 1e-9 * minqddot[6:model.nv]
        stack += 1e-6 * minqddot[model.nv:]


#Base
    if i == Ns-1:
        cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(i).model, dvariables.getVariable("dq"), "base")
        cartesian_task.setWeight(1e-0 * np.eye(6))
        costs.append(cartesian_task)
        base_ref = cartesian_task.getReference().copy()
        # base_ref.translation[1] += 0.3
        # base_ref.translation[0] -= 0.1
        # base_ref.translation[2] -= 0.05
        base_ref.linear = Rz(np.pi/4)
        cartesian_task.setReference(base_ref)
        stack += cartesian_task%[3,4]


#Postural
    postural = Postural(ocp.stage(i).model)
    postural.setWeight(1e-6 * np.diag(q_weights))
    if i==Ns-1:
        postural.setWeight(1e-0 * np.diag(q_weights))
    postural.setReference(q_val.copy())
    minus.append(postural)
    stack += AffineTask.toAffine(postural, dvariables.getVariable("dq"))[6:]

#Compute Torques
    if i<Ns-1:
        tau_compute = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in frame_contact_seq[i]:
            tau_compute.addForce(frame, contact_frames_vars[frame])
        tau_compute.setWeight(0 * np.eye(ocp.stage(i).model.nv))
        mintaus.append(tau_compute)
        stack += tau_compute


    ocp.stage(i).stack = pysot.AutoStack(stack)
    


#Joint Limits
    qmin, qmax = model.getJointLimits()
    qlims_i = JointLimits(ocp.stage(i).model, qmax, qmin)
    qlims.append(qlims_i)
    ocp.stage(i).stack = ocp.stage(i).stack << AffineConstraint.toAffine(qlims_i, dvariables.getVariable("dq"))


    if i < Ns-1:
# Dynamics 
        tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in frame_contact_seq[i]:
            tau_lim.addForce(frame, contact_frames_vars[frame])
        tau_lims = tau_lim.getTorqueLimit()
        tau_lims[:6] = [1e-9]*6
        tau_lim.setTorqueLimit(tau_lims)
        const.append(tau_lim)
        ocp.stage(i).stack = ocp.stage(i).stack << tau_lim


        for frame in frame_contact_seq[i]:
            friction_const = FrictionConeConstraint(ocp.stage(i).model, frame, contact_frames_vars[frame], ocp.stage(i).dx, ocp.stage(i).du)
            const.append(friction_const)
            ocp.stage(i).stack = ocp.stage(i).stack << friction_const

        for frame in contact_frames:
            contact_task = ContactConstraint(ocp.stage(i).model, frame, ocp.stage(i).dx, ocp.stage(i).du)
            costs.append(contact_task)
            if frame in frame_contact_seq[i]:
                contact_task.activate(0.)
            ocp.stage(i).stack = ocp.stage(i).stack << contact_task
    

ocp.update(x0, u0)

print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 100
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1E-4
solver.getOptions().min_abs_delta_solution = 1e-2
solver.getOptions().hessian_scale_factor_up = 1e6

# solver.getQPSolver().getOptions().mode = pysot.HpipmMode.Speed
solver.getQPSolver().getOptions().iter_max = 100

solver.getQPSolver().getOptions().tol_ineq = 1e-2
solver.getQPSolver().getOptions().tol_eq = 1e-2
solver.getQPSolver().getOptions().tol_stat = 1e-2
solver.getQPSolver().getOptions().tol_comp = 1e-2

solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")



ocp.update(x0, u0)
success = solver.solve(x0, u0)

x0 = solver.getStateSolution()
u0 = solver.getControlSolution()

# for i in range(len(x0)-1):
#     # print(f"x0: {x0}")
#     print(f"u0[{i}]:\t {u0[i]}")


msg = JointState()
msg.name = model.getJointNames()[1:]
t= 0.
try:
    while rclpy.ok():
        input()
        x = x0[0]
        for i in range(len(x0)-1):
            x = x0[i]
            q_val = x.tolist()[7:model.nq]
            v_val = x.tolist()[6:model.nv]
            tau_val = mintaus[i].getb()[6:model.nv]
            # print(tau_val)

            msg.header.stamp = ros2node.get_clock().now().to_msg()
            msg.position = q_val
            msg.velocity = v_val
            msg.effort = tau_val

            ros2node.publish(msg)
            time.sleep(DT)

        rclpy.spin_once(ros2node, timeout_sec=0.0)
        

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
