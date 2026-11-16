import viser
import numpy as np

class collision_distances:
    def __init__(self, server):
        self.collision_distances = None
        self.server = server

        with server.gui.add_folder("Collision Distances"):
            self.is_active_checkbox = server.gui.add_checkbox("/Enable", initial_value=True)
            self.is_active_checkbox.on_update(self.on_checkbox_change())

    def update(self, points):
        ch = np.array(points)

        if not self.collision_distances and self.is_active_checkbox.value:
            self.collision_distances = self.server.scene.add_line_segments(
                "/collision_distances",
                points=ch,
                line_width=3,
                colors=(0, 255, 0),
            )
        elif self.is_active_checkbox.value:
            self.collision_distances.points = ch

    def on_checkbox_change(self):
        def _(_):
            is_active = self.is_active_checkbox.value
            if not is_active:
                self.collision_distances.remove()
                self.collision_distances = None
        return _
