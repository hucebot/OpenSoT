import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi
import pyopensot as pysot
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import subprocess
import time
from pyopensot.tasks.velocity import Cartesian
from pyopensot.constraints.velocity import JointLimits, VelocityLimits, ConvexHull, CartesianPositionConstraint
import array
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker, MarkerArray
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA


class ros2_node(Node):
    def __init__(self):
        super().__init__('quadruped_ik')
        self.get_logger().info("quadruped_ik node has been started.")
        self.client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')

        request = GetParameters.Request()
        request.names = ['robot_description']

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.urdf = None
        if future.result() is not None:
            values = future.result().values
            for val in values:
                self.urdf = val.string_value
        else:
            self.get_logger().error('Failed to call service')

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.base_link_broadcaster = TransformBroadcaster(self)

        self.server = InteractiveMarkerServer(self, 'six_dof_marker_server')
        self.marker_pose = PoseStamped()

        self.com_publisher = self.create_publisher(Marker, '/com', 10)
        self.ch_publisher = self.create_publisher(Marker, '/ch', 10)
        self.planes_publisher = self.create_publisher(MarkerArray, '/planes', 10)

    def make_6dof_marker(self, name, pose, frame_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = '6-DOF Control'
        int_marker.scale = 0.3

        int_marker.pose.position.x = pose.translation[0]
        int_marker.pose.position.y = pose.translation[1]
        int_marker.pose.position.z = pose.translation[2]

        quat_xyzw = R.from_matrix(pose.linear).as_quat() # Format: [x, y, z, w]
        int_marker.pose.orientation.x = quat_xyzw[0]
        int_marker.pose.orientation.y = quat_xyzw[1]
        int_marker.pose.orientation.z = quat_xyzw[2]
        int_marker.pose.orientation.w = quat_xyzw[3]

        self.marker_pose.pose = int_marker.pose

        # Add a visible marker (e.g., a cube)
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.05
        cube_marker.scale.y = 0.05
        cube_marker.scale.z = 0.05
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube_marker)
        int_marker.controls.append(control)

        # Add 6-DOF controls
        self.add_6dof_controls(int_marker)


        self.server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        self.marker_pose.header = feedback.header
        self.marker_pose.pose = feedback.pose
    def add_6dof_controls(self, marker):
        axes = ['x', 'y', 'z']
        for axis in axes:
            # Rotation
            control = InteractiveMarkerControl()
            control.name = f'rotate_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)

            # Translation
            control = InteractiveMarkerControl()
            control.name = f'move_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

    def toJointStateMsg(self, joint_state_msg, q):
        joint_state_msg.position = array.array('d', q[7:])

    def publish(self, joint_state_msg, transform_msg):
        self.joint_state_publisher.publish(joint_state_msg)
        self.base_link_broadcaster.sendTransform(transform_msg)


    def publish_com(self, com_position):
        marker = Marker()

        # Header
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        # Namespace and ID
        marker.ns = "sphere"
        marker.id = 0

        # Marker type
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Pose
        marker.pose.position.x = com_position[0]
        marker.pose.position.y = com_position[1]
        marker.pose.position.z = com_position[2]
        marker.pose.orientation.w = 1.0

        # Scale (diameter in meters)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Color (RGBA)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        self.com_publisher.publish(marker)

    def publish_ch(self, ch_points):
        marker = Marker()

        # Header
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "polygon"
        marker.id = 1

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Identity pose (points are already in frame coordinates)
        marker.pose.orientation.w = 1.0

        # Line width (meters)
        marker.scale.x = 0.01

        # Color
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        # Convert points
        marker.points = []

        for ch_point in ch_points:
            p = Point()
            p.x = ch_point[0]
            p.y = ch_point[1]
            p.z = ch_point[2]
            marker.points.append(p)

        # Close polygon by repeating first point
        if len(ch_points) > 2:
            p = Point()
            p.x = ch_points[0][0]
            p.y = ch_points[0][1]
            p.z = ch_points[0][2]
            marker.points.append(p)

        self.ch_publisher.publish(marker)

    def create_plane_marker(self, normal, b, frame_id, marker_id):
        n = normal
        norm = np.linalg.norm(n)
        # point on plane
        p0 = b * n / (norm**2)
        # build basis vectors in plane
        v1 = np.cross(n, [1,0,0])
        if np.linalg.norm(v1) < 1e-6:
            v1 = np.cross(n, [0,1,0])

        v1 = v1 / np.linalg.norm(v1)
        v2 = np.cross(n, v1)
        v2 = v2 / np.linalg.norm(v2)

        s = 1.

        p1 = p0 + s*v1 + s*v2
        p2 = p0 + s*v1 - s*v2
        p3 = p0 - s*v1 - s*v2
        p4 = p0 - s*v1 + s*v2

        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "planes"
        marker.id = marker_id

        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 0.6

        def pt(p):
            P = Point()
            P.x, P.y, P.z = p.tolist()
            return P

        marker.points = [
            pt(p1), pt(p2), pt(p3),
            pt(p1), pt(p3), pt(p4)
        ]

        return marker


    def publish_planes(self, A, b, frame_id):
        msg = MarkerArray()
        for i in range(A.shape[0]):
            normal = A[i]
            bi = b[i]

            marker = self.create_plane_marker(normal, bi, frame_id, i+100)
            msg.markers.append(marker)

        self.planes_publisher.publish(msg)



package_path = None
try:
    package_path = get_package_share_directory('LittleDog')
    print(f"Package path: {package_path}")
except:
    print("To run this example is needed the LittleDog package ([ros2 branch]) that can be download here: https://github.com/EnricoMingo/LittleDog")

roslaunch = subprocess.Popen(['ros2', 'launch', 'LittleDog', 'LittleDog.launch'], stdout=subprocess.PIPE, shell=False)
rviz_file_path = package_path + "/launch/LittleDog.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2',  '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rclpy.init()
node = ros2_node()

model = xbi.ModelInterface2(node.urdf)
qmin, qmax = model.getJointLimits()
dqmax = model.getVelocityLimits()
q = [0., 0., 0.15, 0., 0., 0., 1.,
     0., -0.7, 1.4, 0., -0.7, 1.4, 0., 0.7, -1.4, 0., 0.7, -1.4]
model.setJointPosition(q)
model.update()

msg = JointState()
msg.name = model.getJointNames()[1::]
msg.position = [0.0] * len(msg.name)

w_T_b = TransformStamped()
w_T_b.header.frame_id = "world"
w_T_b.child_frame_id = "body"

dt = 1./100.

contact_frames = ["front_left_foot_center", "front_right_foot_center", "back_left_foot_center", "back_right_foot_center"]

contact_tasks = {}
for contact_frame in contact_frames:
    contact_tasks[contact_frame] = Cartesian(contact_frame, model, contact_frame, "world")

base_task = Cartesian("base", model, "body", "world")

# CONSTRAINTS
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
#
dqmax = model.getVelocityLimits()
dqlims = VelocityLimits(model, dqmax, dt)

convex_hull = ConvexHull(model, contact_frames)

# Planes to constraint the base movement on z
A = np.array([[0., 0., 1.], [0., 0., -1.]])
b = np.array([q[2] + 0.01, -q[2] + 0.05])
base_pos_limits = CartesianPositionConstraint(base_task, A, b)

# STACK
stack = ((contact_tasks[contact_frames[0]][0:3] + contact_tasks[contact_frames[1]][0:3] + contact_tasks[contact_frames[2]][0:3] + contact_tasks[contact_frames[3]][0:3])/base_task) << qlims << dqlims << convex_hull << base_pos_limits
stack.update()

# SOLVER
solver = pysot.iHQP(stack)

pose_ref, vel_ref = base_task.getReference()
node.make_6dof_marker(name="body", pose=pose_ref, frame_id="world")

try:
    while rclpy.ok():
        # Update actual position in the model
        model.setJointPosition(q)
        model.update()

        # Update ref
        pose_ref.translation[0] = node.marker_pose.pose.position.x
        pose_ref.translation[1] = node.marker_pose.pose.position.y
        pose_ref.translation[2] = node.marker_pose.pose.position.z
        quat = [node.marker_pose.pose.orientation.x, node.marker_pose.pose.orientation.y,
                node.marker_pose.pose.orientation.z, node.marker_pose.pose.orientation.w]
        pose_ref.linear = R.from_quat(quat).as_matrix()

        base_task.setReference(pose_ref, vel_ref)


        # Update Stack
        stack.update()
        convex_hull.update()

        # Solve
        dq = solver.solve()
        q = model.sum(q, dq)  # we use the model sum to account for the floating-base and wheels manifolds

        # SSend results to ROS2
        node.toJointStateMsg(msg, q)
        msg.header.stamp = node.get_clock().now().to_msg()
        #
        w_T_b.header.stamp = msg.header.stamp
        w_T_b.transform.translation.x = q[0]
        w_T_b.transform.translation.y = q[1]
        w_T_b.transform.translation.z = q[2]
        w_T_b.transform.rotation.x = q[3]
        w_T_b.transform.rotation.y = q[4]
        w_T_b.transform.rotation.z = q[5]
        w_T_b.transform.rotation.w = q[6]
        #

        com = model.getCOM()
        node.publish_com(np.array([com[0], com[1], 0.]))

        success, ch = convex_hull.getConvexHull()
        for i in range(len(ch)): #express ch in world frame
            ch[i] += np.array([com[0], com[1], com[2]])
        if success:
            node.publish_ch(ch)
        else:
            print("Convex Hull computation failed.")

        node.publish_planes(A, b, "world")

        rclpy.spin_once(node, timeout_sec=0.0)
        node.publish(msg, w_T_b)

        #
        time.sleep(dt)

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    roslaunch.kill()
    rviz.kill()
    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()
