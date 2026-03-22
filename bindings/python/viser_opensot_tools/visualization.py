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

