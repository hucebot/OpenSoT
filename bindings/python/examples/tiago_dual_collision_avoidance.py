from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision
from xbot2_interface import pyaffine3
import time
import numpy as np
import array

import pyopensot as pysot
from pyopensot.tasks.velocity import Postural, Cartesian, Gaze
from pyopensot.constraints.velocity import JointLimits, VelocityLimits, MechanumWheels4X
from pyopensot_collision.constraints.velocity import CollisionAvoidance

import replay
import threading
import json
from pathlib import Path

from scipy.spatial.transform import Rotation as R


def obstacle_box(server):
    box = server.scene.add_box(
        name="obstacle",
        dimensions=(0.1, 0.6, 1.4),
        color=(255, 0, 0),
        opacity=0.5)

    # set pose
    box.position = np.array([0.75, 0.0, 0.75])     # xyz
    box.wxyz = np.array([1, 0, 0, 0])
    return box

def toViserCgf(q):
    q_viser = np.zeros((q.shape[0] - 11, 1))
    #wheels
    q_viser[0] = np.arctan2(q[8],  q[7])
    q_viser[1] = np.arctan2(q[10], q[9])
    q_viser[2] = np.arctan2(q[12], q[11])
    q_viser[3] = np.arctan2(q[14], q[13])
    #torso
    q_viser[4] = q[15]
    #head
    q_viser[5:7] = np.array(q[-2:]).reshape(-1,1)
    #arms
    q_viser[7:] = np.array(q[16:-2]).reshape(-1,1)
    return q_viser


resource = "tiago_pro_capsules.rviz";
urdf_path = pysot.find(resource)
print(f"Loading {urdf_path}")

urdf_string = pysot.ReadFile(urdf_path)

model = xbi.ModelInterface2(urdf_string)

# Set a homing configuration
q = np.array([0., 0., 0., .0, 0., 0., 1., # floating_base
     np.cos(0.), np.sin(0.),     # 'wheel_front_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_front_right_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_right_joint'
     0.00,                # 'torso_lift_joint'
     0.00, .5, 0., -np.pi/2., 0.00, np.pi/2., 0., # 'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'
     0.00, .5, 0., -np.pi/2., 0.00, np.pi/2., 0., # 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
     0., 0.]) # 'head_1_joint', 'head_2_joint'
model.setJointPosition(q)
model.update()

rviz = replay.rvizer(urdf_path)
dt = 1./100.

print(f"Viser joint names: {rviz.viser_urdf.get_actuated_joint_names()}")
print(f"Pinocchio joint names: {model.getJointNames()}")

# CREATE OPTIMIZATION PROBLEM
manipulation_base_frame = "world"
#1. TASKS
gripper_left = Cartesian("gripper_left", model, "gripper_left_grasping_link", manipulation_base_frame)
gripper_left.setLambda(0.1)

gripper_right = Cartesian("gripper_right", model, "gripper_right_grasping_link", manipulation_base_frame)
gripper_right.setLambda(0.1)

base = Cartesian("base", model, "base_link", "world")
base.setLambda(0.1)

postural = Postural(model)
postural.setLambda(0.05)
Wpostural = postural.getWeight()
Wpostural[0:6] = 0.0  # Do not penalize floating base
Wpostural[6:10] = 0.0  # Do not penalize wheels
postural.setWeight(Wpostural)

gaze = Gaze("gaze", model, "base_link", "head_front_camera_link")

# CONSTRAINTS
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
qlims.update()
#
dqmax = model.getVelocityLimits()
dqlims = VelocityLimits(model, dqmax, dt)

# Base2D:
base2D = Cartesian("Cartesian", model, "base_link", "world")
base2D.setLambda(0.1)

# Collision Avoidance
collision_avoidance = CollisionAvoidance(model, max_pairs=100, collision_urdf=urdf_string)
collision_avoidance.setBoundScaling(0.1)
collision_avoidance.setLinkPairThreshold(0.01)
collision_avoidance.setDetectionThreshold(-1)
collision_pairs_path = pysot.find("pairs_tiago_pro.json")
with Path(collision_pairs_path).open("r") as f:
    collision_pairs = json.load(f)

collision_set = {tuple(pair) for pair in collision_pairs["collision_list"]}
collision_avoidance.setCollisionList(collision_set)

# MechanumWheels4X constraint
joint_wheels_name = ["wheel_front_left_joint", "wheel_front_right_joint", "wheel_rear_left_joint", "wheel_rear_right_joint"]
l1 = 0.223
l2 = 0.244
wheel_radius = 0.08
MechanumWheels4X = MechanumWheels4X(l1, l2, wheel_radius, joint_wheels_name, "base_link", model)
#MechanumWheels4X.setIsGlobalVelocity(True)

# STACK
stack = ( (gripper_left + gripper_right + base%[0, 1, 5] + gaze) / postural) << qlims[10:] << dqlims << collision_avoidance << base2D%[2, 3, 4] << MechanumWheels4X
stack.update()

# SOLVER
solver = pysot.iHQP(stack)

object_in_scene = rviz.server.gui.add_checkbox("/Obstacle", initial_value=False)

lock = threading.Lock()
replay.interactive_marker(rviz.server, gripper_left, lock, slider_max=1., slider_step=0.1)
replay.interactive_marker(rviz.server, gripper_right, lock, slider_max=1., slider_step=0.1)
replay.interactive_marker(rviz.server, base, lock, slider_max=1., slider_step=0.1)
replay.postural_gui(rviz.server, model, postural, model.getJointNames()[5:], lock)

coll_dist = replay.collision_distances(rviz.server)


obstacle = None
box = None
try:
    while True:
        model.setJointPosition(q)
        model.update()

        gaze_ref = model.getPose("gripper_right_grasping_link", "base_link")
        gaze.setGaze(gaze_ref)

        #Environment Collision
        if object_in_scene.value:
            if not collision_avoidance.setCollisionShapeActive("mybox", True):
                box = pyxbot2_collision.shape.Box()
                box.size = np.array([0.1, 0.6, 1.4])
                w_T_c = pyaffine3.Affine3()
                w_T_c.translation = np.array([0.75, 0.0, 0.75])
                w_T_c.linear = R.from_quat([0., 0., 0., 1.]).as_matrix()
                collision_avoidance.addCollisionShape("mybox", "world", box, w_T_c, [])
        if not object_in_scene.value and box is not None:
            collision_avoidance.setCollisionShapeActive("mybox", False)

        # Update Stack
        stack.update()


        # Solve
        dq = solver.solve()
        q = model.sum(q, dq)  # we use the model sum to account for the floating-base and wheels manifolds

        rviz.update(q=np.append(0., toViserCgf(q)), base=q[:7])

        coll_dist.update(collision_avoidance.getOrderedWitnessPointVector())
        if object_in_scene.value and obstacle is None:
            obstacle = obstacle_box(rviz.server)
        elif not object_in_scene.value and obstacle is not None:
            obstacle.remove()
            obstacle = None

        time.sleep(dt)
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
        print("Stopping the node.")
