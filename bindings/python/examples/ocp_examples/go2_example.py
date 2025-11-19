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


# print(model.getPose("RL_foot"))
# print(model.getPose("FL_foot"))
# print(model.getPose("RR_foot"))
# print(model.getPose("FR_foot"))

contact_frames = ["RL_foot","FL_foot","RR_foot","FR_foot"]

rclpy.init()
ros2node = ros2_node()

forcesnode = force_node()
forcesnode.initialize_force_publishers(contact_frames)

ros2node.joint_msg.name = model.getJointNames()[1::]
ros2node.publish(q_val)
time.sleep(0.5)

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

x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

dx = AffineHelper.pile(dq, dqdot)
dxdot = AffineHelper.pile(dqdot, dqddot)

contact_frames_vars = {}
contact_frames_dvars = {}
for frame in contact_frames:
    contact_frames_vars[frame] = variables.getVariable(frame+"_force")
    contact_frames_dvars[frame] = dvariables.getVariable(frame+"_dforce")

_u = qddot
for frame in contact_frames:
    _u = AffineHelper.pile(_u, contact_frames_vars[frame])

_du = dqddot
for frame in contact_frames:
    _du = AffineHelper.pile(_du, contact_frames_dvars[frame])

Ns = 30 # number of nodes
tf = 2. # final time
dt = tf/Ns 
print(f"Ns: {Ns}, tf: {tf}, dt: {dt}")

mass = model.getMass()
print(mass)
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
    stage.x = x
    stage.xdot = xdot
    stage.dx = dx

    if i<Ns-1:
        """ We include both control variables and dvariables """
        stage.u =  _u
        stage.du = _du

        stage.variables = contact_frames_vars

    """ We include q and qdot defined for the state variables """
    stage.q = q
    stage.v = qdot
    stage.a = qddot

    


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

alpha = 0.01

costs = []
mintaus = []
stack = None
for i in range(Ns):

    if i < Ns-1:
        minqddot = min_var.create(f"minqddot{i}", ocp.stage(i).u, ocp.stage(i).du)
        minqddot.setWeight(1e-9 * np.eye(model.nv + 4*3))
        costs.append(minqddot)

        minf = min_var.create(f"minf{i}", ocp.stage(i).u, ocp.stage(i).du)
        minf.setWeight(1e-6 * np.eye(model.nv + 4*3))
        costs.append(minf)
        

        mintau = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in contact_frames:
            mintau.addForce(frame, contact_frames_vars[frame])
        mintau.setWeight(1e3 * np.eye(model.nv))
        mintaus.append(mintau)

        stack = minqddot



    cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(i).model, dvariables.getVariable("dq"), "base")
    cartesian_task.setWeight(1e-9 * np.eye(6))
    costs.append(cartesian_task)

    base_ref = cartesian_task.getReference().copy()
    # base_ref.translation[0] = com0[0] + alpha * np.sin(np.pi * i*dt)
    # base_ref.translation[1] = base_ref.translation[1] + alpha * np.cos(np.pi * i*dt)
    # base_ref.translation[2] = base_ref.translation[2] + alpha * np.sin(np.pi * i*dt)
    # cartesian_task.setReference(base_ref)

    stack += cartesian_task

    minvel = min_var.create(f"minvel", ocp.stage(i).x[model.nq:], dvariables.getVariable("dqdot"))
    minvel.setWeight(1e-9 *  np.eye(model.nv))
    costs.append(minvel)

    # postural = Postural(ocp.stage(i).model)
    # postural.setWeight(1e3 * np.eye(model.nv))
    # postural.setReference(q_val.copy())
    stack += minvel


    
    for frame in contact_frames:
        
        # if i<Ns-1:
        #     contact_task = ContactConstraint(ocp.stage(i).model, frame, ocp.stage(i).dx, ocp.stage(i).du)
        #     costs.append(contact_task)
        #     stack += contact_task

        cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(i).model, dvariables.getVariable("dq"), frame)
        cartesian_task.setWeight(1e0 * np.eye(6))
        costs.append(cartesian_task)
        stack += cartesian_task%[0, 1, 2]

    ocp.stage(i).stack = pysot.AutoStack(stack)

    if i < Ns-1:

        for frame in contact_frames:
            contact_task = ContactConstraint(ocp.stage(i).model, frame, ocp.stage(i).dx, ocp.stage(i).du)
            costs.append(contact_task)
            ocp.stage(i).stack <<  contact_task

        # tau_min
        tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
        for frame in contact_frames:
            tau_lim.addForce(frame, contact_frames_vars[frame])

        tau_lims = tau_lim.getTorqueLimit()
        tau_lims[:6] = [1e-9]*6
        tau_lim.setTorqueLimit(tau_lims)
        const.append(tau_lim)
        ocp.stage(i).stack << tau_lim
    

ocp.update(x0, u0)

print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 100
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1e-3
solver.getOptions().min_abs_delta_solution = 1e-3
solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")



ocp.update(x0, u0)
success = solver.solve(x0, u0)

x0 = solver.getStateSolution()
u0 = solver.getControlSolution()

force_msgs = {}
for contact_frame in contact_frames:
    force_msgs[contact_frame] = WrenchStamped()
    force_msgs[contact_frame].header.frame_id = contact_frame
    force_msgs[contact_frame].wrench.torque.x = force_msgs[contact_frame].wrench.torque.y = force_msgs[contact_frame].wrench.torque.z = 0.


try:
    t= 0.
    while rclpy.ok():
        input()

        x = x0[0]
        for i in range(len(x0)):
            x = x0[i]
            q_val = x.tolist()[:model.nq]
            if i<len(u0):
                print(-mintaus[i].getb()[:6])
                #print(const[i].getbLowerBound()[:6])
                #print(const[i].getbUpperBound()[:6])
            ros2node.publish(q_val)

            if i<len(u0):
                j=0
                for contact_frame in contact_frames:
                    T = ocp.stage(i).model.getPose(contact_frame)
                    # force_msgs[contact_frame].header.stamp = msg.header.stamp
                    # f_local = T.linear.transpose() @ variables.getVariable(contact_frame).getValue(x)
                    f_local = u0[i][model.nv + j*3: model.nv + j*3+3]
                    #f_local = T.linear.transpose() @ f_local
                    force_msgs[contact_frame].wrench.force.x = f_local[0]
                    force_msgs[contact_frame].wrench.force.y = f_local[1]
                    force_msgs[contact_frame].wrench.force.z = f_local[2]
                    j+=1

                forcesnode.publish(force_msgs)


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
    # roslaunch.kill()
    ros2node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()



