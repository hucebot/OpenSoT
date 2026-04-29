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


# class ros2_node(Node):
#     def __init__(self):
#         super().__init__('tiago_dual_collision_avoidance')
#         self.get_logger().info("tiago_dual node has been started.")
#         self.client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')

#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for parameter service...')

#         request = GetParameters.Request()
#         request.names = ['robot_description']

#         future = self.client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)

#         self.urdf = None
#         if future.result() is not None:
#             values = future.result().values
#             for val in values:
#                 self.urdf = val.string_value
#         else:
#             self.get_logger().error('Failed to call service')

#         self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
#         self.base_link_broadcaster = TransformBroadcaster(self)

#         self.server = InteractiveMarkerServer(self, 'six_dof_marker_server')
#         self.marker_pose = PoseStamped()

#         self.collision_distances_publisher = self.create_publisher(Marker, 'collision_distances', 10)

#         self.srv = self.create_service(SetBool, 'enable_external_obstacle', self.handle_request) # ros2 service call /enable_external_obstacle std_srvs/srv/SetBool "{data: True}"
#         self.enable_external_obstacle = False
#         self.world_object_publisher = self.create_publisher(Marker, 'world_object', 10)

#         self.cube = Marker()
#         self.cube.header.frame_id = "world"
#         self.cube.ns = "environment"
#         self.cube.id = 0
#         self.cube.type = Marker.CUBE
#         self.cube.scale.x = 0.1
#         self.cube.scale.y = 0.6
#         self.cube.scale.z = 1.4
#         self.cube.color.g = 1.0
#         self.cube.color.a = 0.5
#         self.cube.pose.position.x = 0.75
#         self.cube.pose.position.y = 0.0
#         self.cube.pose.position.z = 0.75
#         self.cube.pose.orientation.x = self.cube.pose.orientation.y = self.cube.pose.orientation.z = 0.0
#         self.cube.pose.orientation.w = 1.0

#     def handle_request(self, request, response):
#         self.get_logger().info(f"Received request: enable_external_obstacle = {request.data}")
#         self.enable_external_obstacle = request.data
#         response.success = True
#         response.message = f"Received {request.data}"
#         return response

#     def publishObstacle(self, time, action):
#         if self.enable_external_obstacle:
#             self.cube.header.stamp = time
#             self.cube.action = action
#             self.world_object_publisher.publish(self.cube)


#     def make_6dof_marker(self, name, pose, frame_id):
#         int_marker = InteractiveMarker()
#         int_marker.header.frame_id = frame_id
#         int_marker.name = name
#         int_marker.description = '6-DOF Control'
#         int_marker.scale = 0.3

#         int_marker.pose.position.x = pose.translation[0]
#         int_marker.pose.position.y = pose.translation[1]
#         int_marker.pose.position.z = pose.translation[2]

#         quat_xyzw = R.from_matrix(pose.linear).as_quat() # Format: [x, y, z, w]
#         int_marker.pose.orientation.x = quat_xyzw[0]
#         int_marker.pose.orientation.y = quat_xyzw[1]
#         int_marker.pose.orientation.z = quat_xyzw[2]
#         int_marker.pose.orientation.w = quat_xyzw[3]

#         self.marker_pose.pose = int_marker.pose

#         # Add a visible marker (e.g., a cube)
#         cube_marker = Marker()
#         cube_marker.type = Marker.CUBE
#         cube_marker.scale.x = 0.05
#         cube_marker.scale.y = 0.05
#         cube_marker.scale.z = 0.05
#         cube_marker.color.r = 0.0
#         cube_marker.color.g = 1.0
#         cube_marker.color.b = 0.0
#         cube_marker.color.a = 1.0

#         control = InteractiveMarkerControl()
#         control.always_visible = True
#         control.markers.append(cube_marker)
#         int_marker.controls.append(control)

#         # Add 6-DOF controls
#         self.add_6dof_controls(int_marker)


#         self.server.insert(marker=int_marker, feedback_callback=self.process_feedback)
#         self.server.applyChanges()
#     def process_feedback(self, feedback):
#         self.marker_pose.header = feedback.header
#         self.marker_pose.pose = feedback.pose
#     def add_6dof_controls(self, marker):
#         axes = ['x', 'y', 'z']
#         for axis in axes:
#             # Rotation
#             control = InteractiveMarkerControl()
#             control.name = f'rotate_{axis}'
#             control.orientation.w = 1.0
#             setattr(control.orientation, axis, 1.0)
#             control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
#             marker.controls.append(control)

#             # Translation
#             control = InteractiveMarkerControl()
#             control.name = f'move_{axis}'
#             control.orientation.w = 1.0
#             setattr(control.orientation, axis, 1.0)
#             control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
#             marker.controls.append(control)

#
#     def publish(self, joint_state_msg, transform_msg):
#         self.joint_state_publisher.publish(joint_state_msg)
#         self.base_link_broadcaster.sendTransform(transform_msg)

#     def publishCollisionDistances(self, collision_distance_points, time):
#         marker = Marker()
#         marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0
#         marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
#         marker.pose.orientation.w = 1.0
#         marker.type = Marker.LINE_LIST
#         marker.action = Marker.ADD
#         marker.header.frame_id = "world"
#         marker.header.stamp = time
#         marker.ns = "collision_distances"
#         marker.id = 0
#         marker.scale.x = 0.005  # Line width
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0  # Opaque

#         for point_pairs in collision_distance_points:
#             pa = point_pairs[0]
#             pb = point_pairs[1]

#             point_a = Point()
#             point_a.x = pa[0]
#             point_a.y = pa[1]
#             point_a.z = pa[2]

#             point_b = Point()
#             point_b.x = pb[0]
#             point_b.y = pb[1]
#             point_b.z = pb[2]

#             marker.points.append(point_a)
#             marker.points.append(point_b)


#         self.collision_distances_publisher.publish(marker)

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
# collision_list = {
#     ("arm_left_3_link", "base_link"),
#     ("arm_left_5_link", "base_link"),
#     ("gripper_left_left_finger_link", "base_link"),
#     ("gripper_left_right_finger_link", "base_link"),
#     ("gripper_left_link", "base_link"),

#     ("arm_left_3_link", "head_2_link"),
#     ("arm_left_5_link", "head_2_link"),
#     ("gripper_left_left_finger_link", "head_2_link"),
#     ("gripper_left_right_finger_link", "head_2_link"),
#     ("gripper_left_link", "head_2_link"),

#     ("arm_left_3_link", "torso_lift_link"),
#     ("arm_left_5_link", "torso_lift_link"),
#     ("gripper_left_left_finger_link", "torso_lift_link"),
#     ("gripper_left_right_finger_link", "torso_lift_link"),
#     ("gripper_left_link", "torso_lift_link"),

#     ("arm_right_3_link", "base_link"),
#     ("arm_right_5_link", "base_link"),
#     ("gripper_right_left_finger_link", "base_link"),
#     ("gripper_right_right_finger_link", "base_link"),
#     ("gripper_right_link", "base_link"),

#     ("arm_right_3_link", "head_2_link"),
#     ("arm_right_5_link", "head_2_link"),
#     ("gripper_right_left_finger_link", "head_2_link"),
#     ("gripper_right_right_finger_link", "head_2_link"),
#     ("gripper_right_link", "head_2_link"),

#     ("arm_right_3_link", "torso_lift_link"),
#     ("arm_right_5_link", "torso_lift_link"),
#     ("gripper_right_left_finger_link", "torso_lift_link"),
#     ("gripper_right_right_finger_link", "torso_lift_link"),
#     ("gripper_right_link", "torso_lift_link"),

#     ("gripper_right_left_finger_link", "gripper_left_left_finger_link"),
#     ("gripper_right_left_finger_link", "gripper_left_right_finger_link"),
#     ("gripper_right_right_finger_link", "gripper_left_left_finger_link"),
#     ("gripper_right_right_finger_link", "gripper_left_right_finger_link"),
#     ("gripper_right_left_finger_link", "gripper_left_link"),
#     ("gripper_right_right_finger_link", "gripper_left_link"),
#     ("gripper_left_left_finger_link", "gripper_right_link"),
#     ("gripper_left_right_finger_link", "gripper_right_link"),
#     ("gripper_left_link", "gripper_right_link"),

#     ("gripper_left_link", "arm_right_5_link"),
#     ("gripper_right_link", "arm_left_5_link"),
#     ("arm_left_5_link", "arm_right_5_link"),
#     ("arm_left_5_link", "arm_right_4_link"),
#     ("arm_left_4_link", "arm_right_5_link"),
#     ("gripper_left_link", "arm_right_4_link"),
#     ("gripper_left_link", "arm_right_5_link"),
#     ("gripper_right_link", "arm_left_4_link"),
#     ("gripper_right_link", "arm_left_5_link"),

#     ("torso_fixed_column_link", "gripper_right_left_finger_link"),
#     ("torso_fixed_column_link", "gripper_right_right_finger_link"),
#     ("torso_fixed_column_link", "gripper_left_left_finger_link"),
#     ("torso_fixed_column_link", "gripper_left_right_finger_link")
# }
# collision_avoidance.setCollisionList(collision_list)

# MechanumWheels4X constraint
joint_wheels_name = ["wheel_front_left_joint", "wheel_front_right_joint", "wheel_rear_left_joint", "wheel_rear_right_joint"]
l1 = 0.223
l2 = 0.244
wheel_radius = 0.08
MechanumWheels4X = MechanumWheels4X(l1, l2, wheel_radius, joint_wheels_name, "base_link", model)
#MechanumWheels4X.setIsGlobalVelocity(True)

# STACK
stack = ( (gripper_left + gripper_right + base%[0, 1, 5] + gaze) / postural) << qlims[10:] << dqlims << base2D%[2, 3, 4] << MechanumWheels4X #collision_avoidance <<
stack.update()

# SOLVER
solver = pysot.iHQP(stack)

lock = threading.Lock()
replay.interactive_marker(rviz.server, gripper_left, lock, slider_max=1., slider_step=0.1)
replay.interactive_marker(rviz.server, gripper_right, lock, slider_max=1., slider_step=0.1)
replay.interactive_marker(rviz.server, base, lock, slider_max=1., slider_step=0.1)


object_in_scene = False
try:
    while True:
        model.setJointPosition(q)
        model.update()

        gaze_ref = model.getPose("gripper_right_grasping_link", "base_link")
        gaze.setGaze(gaze_ref)

#         #Environment Collision
#         if node.enable_external_obstacle:
#             if not collision_avoidance.setCollisionShapeActive("mybox", True):
#                 box = pyxbot2_collision.shape.Box()
#                 box.size = np.array([node.cube.scale.x, node.cube.scale.y, node.cube.scale.z])
#                 w_T_c = pyaffine3.Affine3()
#                 w_T_c.translation = np.array([node.cube.pose.position.x, node.cube.pose.position.y, node.cube.pose.position.z])
#                 w_T_c.linear = R.from_quat([node.cube.pose.orientation.x, node.cube.pose.orientation.y, node.cube.pose.orientation.z, node.cube.pose.orientation.w]).as_matrix()
#                 collision_avoidance.addCollisionShape("mybox", "world", box, w_T_c, [])
#                 object_in_scene = True
#         elif not node.enable_external_obstacle and object_in_scene:
#             collision_avoidance.setCollisionShapeActive("mybox", False)

        # Update Stack
        stack.update()


        # Solve
        dq = solver.solve()
        q = model.sum(q, dq)  # we use the model sum to account for the floating-base and wheels manifolds

        rviz.update(q=np.append(0., toViserCgf(q)), base=q[:7])


#         node.publishCollisionDistances(collision_avoidance.getOrderedWitnessPointVector(), msg.header.stamp)
#         if node.enable_external_obstacle:
#             node.publishObstacle(msg.header.stamp, Marker.ADD)
#         else:
#             node.publishObstacle(msg.header.stamp, Marker.DELETE)

        time.sleep(dt)
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
        print("Stopping the node.")
