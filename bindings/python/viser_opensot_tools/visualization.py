import viser
from yourdfpy import URDF
from viser.extras import ViserUrdf
from scipy.spatial.transform import Rotation as R
import numpy as np

class rvizer:
    def __init__(self, URDF_PATH):
        self.server = viser.ViserServer(verbose=True)

        self.urdf = URDF.load(
                URDF_PATH,
                load_meshes=True,
                build_scene_graph=True,
                load_collision_meshes=True,
                build_collision_scene_graph=True,
            )

        self._root_node_name = "/robot_base"

        self.base = self.server.scene.add_frame(self._root_node_name, show_axes=False)

        self.viser_urdf = ViserUrdf(
                self.server,
                urdf_or_path=self.urdf,
                root_node_name=self._root_node_name,
                load_meshes=True,
                load_collision_meshes=True,
                collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
            )

        self.server.scene.add_grid(
                "/ground_grid",
                width=5.0,
                height=5.0,
                width_segments=20,
                height_segments=20,
            )

        self.server.initial_camera.position = (1.5, 1.5, 1.5)
        self.server.initial_camera.look_at = (0.0, 0.0, 0.5)
        self.server.initial_camera.up = (0.0, 0.0, 1.0)

        # Add coordinate frame for each joint.
        self._joint_frames: List[viser.SceneNodeHandle] = []
        for joint in self.urdf.joint_map.values():
            self._joint_frames.append(
                self.server.scene.add_frame(
                    self._viser_name_from_frame(
                        self.urdf, joint.child, self._root_node_name
                    ),
                    show_axes=True,
                )
            )

        for link_name, link in self.urdf.link_map.items():
            parent_name = self.urdf.scene.graph.transforms.parents.get(link.name, None)
            if parent_name == None:
                continue  # Base link.
            T_parent_child = self.urdf.get_transform(link.name, parent_name)
            viser_link_name = self._viser_name_from_frame(self.urdf, link.name, root_node_name=self._root_node_name)

            quat = R.from_matrix(T_parent_child[:3, :3].copy()).as_quat()  # returns [x, y, z, w]

            self.server.scene.add_frame(
               viser_link_name,
               show_axes=True,
               axes_length=0.1,
               axes_radius=0.01,
               wxyz=np.array([quat[3], quat[0], quat[1], quat[2]]),
               position=T_parent_child[:3, 3] * 1.,
            )

        self.robot_visualization()

        self.contact_force_visualization_is_inited = False

    def _viser_name_from_frame(self, urdf: URDF, frame_name: str, root_node_name: str = "/",) -> str:
        assert root_node_name.startswith("/")
        assert len(root_node_name) == 1 or not root_node_name.endswith("/")

        frames = []
        while frame_name != urdf.scene.graph.base_frame:
            frames.append(frame_name)
            frame_name = urdf.scene.graph.transforms.parents[frame_name]
        if root_node_name != "/":
            frames.append(root_node_name)
        return "/".join(frames[::-1])


    def update(self, q, base=None, contact_forces_dict=None): #contact_forces_dict is a dict of {"frame": values} where values are in world frame and frame is where the force is applied
        with self.server.atomic():
            self.viser_urdf.update_cfg(q)

            if base is not None:
                self.base.position = base[:3]
                self.base.wxyz = np.array([base[6], base[3], base[4], base[5]])

            if contact_forces_dict is not None:
                w_T_b = self.base_to_transform(base)
                scale = 0.01
                segments = []
                for key, value in contact_forces_dict.items():
                    w_T_c = w_T_b @ self.urdf.get_transform(frame_to=key)

                    start = w_T_c[0:3, 3]
                    end = start + scale * value

                    segments.append([start, end])

                if not self.contact_force_visualization_is_inited:
                    self.contact_force_visualization_is_inited = True
                    self.contact_forces_handle = self.server.scene.add_line_segments(
                    "/contact_forces",
                    points=segments,
                    line_width=3,
                    colors=(255, 0, 0)
                    )
                else:
                    self.contact_forces_handle.points = np.array(segments)


            self.update_frame_placement()

        self.server.flush()


    def base_to_transform(self, q):
        """
        q: numpy array [x, y, z, qx, qy, qz, qw]
        returns: 4x4 homogeneous transform
        """
        T = np.eye(4)

        position = q[:3]
        quat = q[3:]

        Rot = R.from_quat(quat).as_matrix()

        T[:3, :3] = Rot
        T[:3, 3] = position

        return T

    def update_frame_placement(self):
        for joint, frame_handle in zip(self.urdf.joint_map.values(), self._joint_frames):
            T_parent_child = self.urdf.get_transform(joint.child, joint.parent)
            quat = R.from_matrix(T_parent_child[:3, :3].copy()).as_quat()  # returns [x, y, z, w]
            frame_handle.wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
            frame_handle.position = T_parent_child[:3, 3] * 1.


    def robot_visualization(self):
        with self.server.gui.add_folder("Visibility"):
            show_meshes_cb = self.server.gui.add_checkbox(
                "Show meshes",
                initial_value=True
            )

            show_collision_meshes_cb = self.server.gui.add_checkbox(
                "Show collision meshes",
                initial_value=False
            )
            self.viser_urdf.show_collision = False

            show_frames_cb = self.server.gui.add_checkbox(
                "Show frames",
                initial_value=True
            )

        @show_meshes_cb.on_update
        def _(_):
            self.viser_urdf.show_visual = show_meshes_cb.value

        @show_collision_meshes_cb.on_update
        def _(_):
            self.viser_urdf.show_collision = show_collision_meshes_cb.value

        @show_frames_cb.on_update
        def _(_):
            for frame in self._joint_frames:
                frame.show_axes = show_frames_cb.value

