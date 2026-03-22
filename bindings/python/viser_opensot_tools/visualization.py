import viser
from yourdfpy import URDF
from viser.extras import ViserUrdf

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

        self.server.scene.add_frame("/robot_base", show_axes=False)
        self.viser_urdf = ViserUrdf(
                self.server,
                urdf_or_path=self.urdf,
                root_node_name="/robot_base",
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

        robot_visualization(self.server, self.viser_urdf)


class robot_visualization:
    def __init__(self, server, viser_urdf):
        with server.gui.add_folder("Visibility"):
            show_meshes_cb = server.gui.add_checkbox(
                "Show meshes",
                initial_value=True
            )

            show_collision_meshes_cb = server.gui.add_checkbox(
                "Show collision meshes",
                initial_value=False
            )
            viser_urdf.show_collision = False

        @show_meshes_cb.on_update
        def _(_):
            viser_urdf.show_visual = show_meshes_cb.value

        @show_collision_meshes_cb.on_update
        def _(_):
            viser_urdf.show_collision = show_collision_meshes_cb.value

