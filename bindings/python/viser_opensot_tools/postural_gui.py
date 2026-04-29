import viser
import numpy as np
from viser.extras import ViserUrdf

class postural_gui():
    def __init__(self, server: viser.ViserServer, viser_urdf: ViserUrdf, postural_task) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
        self.slider_handles: list[viser.GuiInputHandle[float]] = []
        i = 0
        self.initial_config, _ = postural_task.getReference()
        self.postural_task = postural_task

        with server.gui.add_folder(postural_task.getTaskID()):
            for joint_name, (lower,upper,) in viser_urdf.get_actuated_joint_limits().items():
                lower = lower if lower is not None else -np.pi
                upper = upper if upper is not None else np.pi
                slider = server.gui.add_slider(
                    label=joint_name,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=self.initial_config[i])

                slider.on_update(
                    lambda _:
                        self.postural_task.setReference(np.array([slider.value for slider in self.slider_handles]))
                )

                self.slider_handles.append(slider)

                i += 1

            reset_button = server.gui.add_button("Reset")

            @reset_button.on_click
            def _(_):
                self.postural_task.setReference(self.initial_config)
