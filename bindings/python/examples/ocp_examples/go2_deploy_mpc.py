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
from sensor_msgs.msg import JointState, Joy
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


        self.joint_states_subsriber = self.create_subscription(
            JointState,             # message type
            '/joint_states',      # topic name
            self.joint_states_callback,      # callback function
            10                       # QoS (queue size)
        )
        self.get_logger().info('JointSubscriber node has been started.')

        self.joy_subsriber = self.create_subscription(
            Joy,             # message type
            '/joy',      # topic name
            self.joy_callback,      # callback function
            10                       # QoS (queue size)
        )


        self.base_link_broadcaster = TransformBroadcaster(self)
        self.joint_msg = JointState()
        self.w_T_b = TransformStamped()
        self.w_T_b.header.frame_id = "world"
        self.w_T_b.child_frame_id = "commands/base"


        self.urdf=None
        self.state = None
        self.joy_cmd = None
        # while self.urdf is None:
        while self.urdf is None or self.joy_cmd is None:
            rclpy.spin_once(self)

        self.get_logger().info(f"{name} initialization complete")


    def joint_states_callback(self, msg: JointState):
        self.state = np.concatenate((msg.position , msg.velocity))

    def joy_callback(self, msg: Joy):
        self.joy_cmd = msg

    def listener_callback(self, msg):
        self.get_logger().info("URDF readed")
        self.urdf = msg.data

    def publish(self, joint_state_msg:JointState, q):

        self.w_T_b.header.stamp = self.get_clock().now().to_msg()
        self.w_T_b.transform.translation.x = q[0]
        self.w_T_b.transform.translation.y = q[1]
        self.w_T_b.transform.translation.z = q[2]
        self.w_T_b.transform.rotation.x = q[3]
        self.w_T_b.transform.rotation.y = q[4]
        self.w_T_b.transform.rotation.z = q[5]
        self.w_T_b.transform.rotation.w = q[6]
        self.base_link_broadcaster.sendTransform(self.w_T_b)
        self.joint_state_publisher.publish(joint_state_msg)


rclpy.init()
ros2node = ros2_node()
urdf_string = ros2node.urdf
model = xbi.ModelInterface2(urdf_string)


# q_init = ros2node.state[:12]
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

q_weights[8]   = w_elbow * q_weights[8]
q_weights[11]  = w_elbow * q_weights[11]
q_weights[14]  = w_elbow * q_weights[14]
q_weights[17]  = w_elbow * q_weights[17]



q_val = np.concatenate((np.array([0.,0.,0.,0.,0.,0.,1.]),q_init))
qdot_val = np.zeros(model.nv)
qddot_val = np.zeros(model.nv)


model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
qmin, qmax = model.getJointLimits()
model.update()


z = model.getPose("RL_foot_").translation[2]
z += model.getPose("FL_foot_").translation[2]
z += model.getPose("RR_foot_").translation[2]
z += model.getPose("FR_foot_").translation[2]
z = z/4

q_val[2] = -z


model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
qmin, qmax = model.getJointLimits()
model.update()


# print(model.getPose("RL_foot_"))
# print(model.getPose("FL_foot_"))
# print(model.getPose("RR_foot_"))
# print(model.getPose("FR_foot_"))
# input()


contact_frames = ["RL_foot_","FL_foot_","RR_foot_","FR_foot_"]

forcesnode = force_node()
forcesnode.initialize_force_publishers(contact_frames)



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


trajopt_dt = 0.02
trajopt_nodes = 10
mpc_dt = 0.02

contact_scheduler = Scheduler()
contact_scheduler.addContact("rl", ["RL_foot_"])
contact_scheduler.addContact("rr", ["RR_foot_"])
contact_scheduler.addContact("fl", ["FL_foot_"])
contact_scheduler.addContact("fr", ["FR_foot_"])
contact_scheduler.addContact("all", ["FR_foot_", "FL_foot_", "RR_foot_", "RL_foot_"])
contact_scheduler.addContact("fr_air", ["FL_foot_", "RR_foot_", "RL_foot_"])
contact_scheduler.addContact("fl_air", ["FR_foot_", "RR_foot_", "RL_foot_"])
contact_scheduler.addContact("rr_air", ["FR_foot_", "FL_foot_", "RL_foot_"])
contact_scheduler.addContact("rl_air", ["FR_foot_", "FL_foot_", "RR_foot_"])


gaits = ["stance", "walk", "jump", "trot"]

contact_scheduler.addPhase(["all"], .2, sequence_name="stance")

contact_scheduler.addPhase(["all"], .3, sequence_name="jump")
contact_scheduler.addPhase(["air"], .2, sequence_name="jump")

contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["fl_air"], duration=.1, sequence_name="walk")
contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["rr_air"], duration=.1, sequence_name="walk")
contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["fr_air"], duration=.1, sequence_name="walk")
contact_scheduler.addPhase(["all"], .1, sequence_name="walk")
contact_scheduler.addPhase(contacts_list=["rl_air"], duration=.1, sequence_name="walk")

contact_scheduler.addPhase(["all"], .1, sequence_name="trot")
contact_scheduler.addPhase(["rr", "fl"], .1,sequence_name="trot")
contact_scheduler.addPhase(["all"], .1,sequence_name="trot")
contact_scheduler.addPhase(["rl", "fr"], .1, sequence_name="trot")

gait = "stance"


frame_contact_seq = contact_scheduler.getSequence(trajopt_dt, nodes_number = trajopt_nodes, sequence_name=gait)

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
for i in range(trajopt_nodes):
    x0.append(np.concatenate((q_val, qdot_val)))
    if i<trajopt_nodes-1:
        u0.append(qddot_val)
        for frame in contact_frames:
            u0[i] = np.concatenate((u0[i], f0))

ocp = pysot.oc.OCP()
dd = list()
const = list()
minus = list()
for i in range(trajopt_nodes):
    stage = Stage()
    """ First we include information related to the state space """
    stage.state_space = CompositeSpace([SE3Space(), VectorSpace(model.nq-7), VectorSpace(model.nv)])

    """ We include both state variables and dvariables """
    stage.x = x.copy()
    stage.xdot = xdot.copy()
    stage.dx = dx.copy()

    if i<trajopt_nodes-1:
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


for i in range(trajopt_nodes-1):
    dbase =   pysot.oc.EulerSE3(ocp.stage(i).model, dx[:6], dxdot[:6], ocp.stage(i).x[:7], ocp.stage(i).xdot[:6], ocp.stage(i+1).x[:7], trajopt_dt)
    dpos = pysot.oc.EulerVector(ocp.stage(i).model, dx[6:model.nv], dxdot[6:model.nv], ocp.stage(i).x[7:model.nq], ocp.stage(i).xdot[6:model.nv], ocp.stage(i+1).x[7:model.nq], trajopt_dt)
    dvel = pysot.oc.EulerVector(ocp.stage(i).model, dx[model.nv:], dxdot[model.nv:], ocp.stage(i).v, ocp.stage(i).a, ocp.stage(i+1).v, trajopt_dt)
    dd.append(dbase)
    dd.append(dpos)
    dd.append(dvel)
    ocp.stage(i).dynamics_derivative = dbase + dpos + dvel

ocp.update(x0, u0)


costs = []
calctaus = []
qlims = list()

cost_list = []
constraints = []
for i in range(trajopt_nodes):
    cost_list.append({})
    stack = None

    minvel = min_var.create(f"minvel", ocp.stage(i).x[model.nq:],  ocp.stage(i).dx[model.nv:])
    minvel.setWeight(1e-6 *  np.eye(model.nv))
    # if i==trajopt_nodes-1:
    #     minvel.setWeight(1e3  *  np.eye(model.nv))
    costs.append(minvel)
    stack = minvel[6:model.nv]

    if i < trajopt_nodes-1:
        minqddot = min_var.create(f"minqddot{i}", ocp.stage(i).u, ocp.stage(i).du)
        minqddot.setWeight(np.eye(model.nv + 4*3))
        costs.append(minqddot)
        stack += 1e-6 * minqddot[6:model.nv]
        stack += 1e-8 * minqddot[model.nv:]


# Base
    if i == trajopt_nodes-1:
        cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(i).model,  ocp.stage(i).dx, "base")
        cartesian_task.setWeight(1e-0 * np.eye(6))
        costs.append(cartesian_task)
        base_ref = cartesian_task.getReference().copy()
        cartesian_task.setReference(base_ref)
        # stack += cartesian_task%[3,4,5]

# Base velocity
    if i <= trajopt_nodes-1:
        cartesian_vel_task = pysot.oc.SE3VelTask("Cartesian", ocp.stage(i).model,  ocp.stage(i).dx, "base")
        cartesian_vel_task.setReferenceVelocity([.0,-0.,0.,0.,0.,0.])
        cartesian_vel_task.setWeight(1e-2 * np.eye(6))
        cost_list[i]["base_vel"] = cartesian_vel_task
        stack += cartesian_vel_task



# Postural
    postural = Postural(ocp.stage(i).model)
    postural.setWeight(1e-6 * np.diag(q_weights))
    # if i == trajopt_nodes-1: postural.setWeight(1e0 * np.diag(q_weights))
    postural.setReference(q_val.copy())
    minus.append(postural)
    # stack += AffineTask.toAffine(postural, dvariables.getVariable("dq"))[2]
    # stack += AffineTask.toAffine(postural, dvariables.getVariable("dq"))[6:]


#Compute Torques
    if i<trajopt_nodes-1:
        tau_compute = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in frame_contact_seq[i]:
            tau_compute.addForce(frame, contact_frames_vars[frame])
        tau_compute.setWeight(0 * np.eye(ocp.stage(i).model.nv))
        calctaus.append(tau_compute)
        # stack += tau_compute

#Feet air
    cost_list[i]["feet_height"] = {}
    if i<trajopt_nodes-1:
        for frame in contact_frames:
            cartesian_task = Cartesian("Cartesian", ocp.stage(i).model, frame, "world")
            cartesian_task.setLambda(1)
            cartesian_task.setWeight(1e-0 * np.eye(6))
            cost_list[i]["feet_height"][frame] = cartesian_task
            # stack += AffineTask.toAffine(cartesian_task, dvariables.getVariable("dq"))%[2]


    ocp.stage(i).stack = pysot.AutoStack(stack)    

#Joint Limits
    # qlims_i = JointLimits(ocp.stage(i).model, qmax, qmin)
    # qlims.append(qlims_i)
    # ocp.stage(i).stack = ocp.stage(i).stack << AffineConstraint.toAffine(qlims_i, dvariables.getVariable("dq"))


    constraints.append({})
    if i < trajopt_nodes-1:
# Dynamics 
        tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in frame_contact_seq[i]:
            tau_lim.addForce(frame, contact_frames_vars[frame])
        tau_lims = tau_lim.getTorqueLimit()
        tau_lims[:6] = [1e-9]*6
        tau_lim.setTorqueLimit(tau_lims)
        constraints[i]["dynamics"] = tau_lim
        ocp.stage(i).stack = ocp.stage(i).stack << tau_lim

        
        for frame in contact_frames:
            friction_const = FrictionConeConstraint(ocp.stage(i).model, frame, contact_frames_vars[frame], ocp.stage(i).dx, ocp.stage(i).du)
            friction_const.setCoefficient(0.9)
            const.append(friction_const)
            ocp.stage(i).stack = ocp.stage(i).stack << friction_const

        constraints[i]["friction"] = {}
        for frame in contact_frames:
            contact_task = ContactConstraint(ocp.stage(i).model, frame, ocp.stage(i).dx, ocp.stage(i).du)
            costs.append(contact_task)
            constraints[i]["friction"][frame] = contact_task
            if frame in frame_contact_seq[i]:
                contact_task.activate(0.)
            ocp.stage(i).stack = ocp.stage(i).stack << contact_task


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
# solver.getQPSolver().getOptions().iter_max = 4

solver.getQPSolver().getOptions().tol_ineq = 1e-4
solver.getQPSolver().getOptions().tol_eq = 1e-4
solver.getQPSolver().getOptions().tol_stat = 1e-3
solver.getQPSolver().getOptions().tol_comp = 1e-3

solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")



ocp.update(x0, u0)
success = solver.solve(x0, u0)
x0 = solver.getStateSolution()
u0 = solver.getControlSolution()

# solver.getQPSolver().getOptions().mode = pysot.HpipmMode.Speed
solver.getOptions().verbose = 1
solver.getOptions().line_search_strategy = 2
solver.getQPSolver().getOptions().iter_max = 10
solver.getOptions().wall_time = mpc_dt
solver.getOptions().max_iters = 4
solver.init()

print("inited")
input()

force_msgs = {}
for contact_frame in contact_frames:
    force_msgs[contact_frame] = WrenchStamped()
    force_msgs[contact_frame].header.frame_id = "commands/"+contact_frame
    force_msgs[contact_frame].wrench.torque.x = force_msgs[contact_frame].wrench.torque.y = force_msgs[contact_frame].wrench.torque.z = 0.


h_feet_target,_ = cost_list[0]["feet_height"][contact_frames[0]].getReference()
h_feet_target.translation[2] = 0.05

q_space =  CompositeSpace([SE3Space(), VectorSpace(model.nq-7)])



msg = JointState()
msg.name = model.getJointNames()[1:]
try:
    t= 0.
    while rclpy.ok():
        
        wz = 0.5 * ros2node.joy_cmd.axes[2]
        vx = 0.3 * ros2node.joy_cmd.axes[1]
        vy = 0.3 * ros2node.joy_cmd.axes[0]
        vz = 0.1 * ros2node.joy_cmd.axes[3]

        frame_contact_seq = contact_scheduler.getSequence(trajopt_dt,sequence_name=gait, nodes_number = trajopt_nodes, current_time = t)

        for i in range(trajopt_nodes-1):
            cost_list[i]["base_vel"].setReferenceVelocity([vx,vy,vz,0.,0.,wz])
            for frame in contact_frames:
                if frame in frame_contact_seq[i]:
                    constraints[i]["friction"][frame].activate(0.)
                    constraints[i]["dynamics"].addForce(frame, contact_frames_vars[frame])
                    h_feet_target.translation[2] = 0.
                    cost_list[i]["feet_height"][frame].setReference(h_feet_target)
                    calctaus[i].addForce(frame, contact_frames_vars[frame])
                else:
                    constraints[i]["friction"][frame].deactivate()
                    h_feet_target.translation[2] = 0.
                    cost_list[i]["feet_height"][frame].setReference(h_feet_target)
                    constraints[i]["dynamics"].removeForce(frame)
                    calctaus[i].removeForce(frame)

        # x0[1][7:model.nq] = ros2node.state[:12]
        # x0[0][model.nq+6:] = ros2node.state[12:]

        ocp.update(x0, u0)
        solve = solver.solve(x0, u0)
        x0 = solver.getStateSolution()
        u0 = solver.getControlSolution()

        v_val = x0[0][model.nq:]+ u0[0][:model.nv]*mpc_dt
        q_val = q_space.plus(x0[0][:model.nq], v_val*mpc_dt) 


        # x = x0[1].tolist()
        # q_val = x[7:model.nq]
        # v_val = x[model.nq+6:]
        tau_val = - calctaus[0].getb()[6:model.nv]

        msg.header.stamp = ros2node.get_clock().now().to_msg()
        msg.position = q_val[7:model.nq]
        msg.velocity = v_val[6:model.nv]
        msg.effort = tau_val

        if solve:
            ros2node.publish(msg, q_val.tolist())
        
        j=0
        i=0
        for contact_frame in contact_frames:
            T = ocp.stage(i).model.getPose(contact_frame)
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
        u0[-1] = u0[-1]*0.

        # x0[0] = np.concatenate((q_val,v_val))
        
        t += mpc_dt
       
        rclpy.spin_once(ros2node, timeout_sec=0.1)  

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
