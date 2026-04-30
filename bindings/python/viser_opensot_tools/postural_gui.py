import viser
import numpy as np
from viser.extras import ViserUrdf
from itertools import islice

def model_2_viser(viser_urdf, model):
    print(dir(model))
    print(model.getJointNames())
    print(model.getJointInfo("wheel_front_left_joint"))

class postural_gui():
    def __init__(self, server: viser.ViserServer, model, postural_task, dof_list, lock) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
        self.slider_handles: list[viser.GuiInputHandle[float]] = []
        self.initial_config, _ = postural_task.getReference()
        self.postural_task = postural_task
        self.model = model
        self.dof_list = dof_list
        self.lock = lock

        lowers, uppers = self.model.getJointLimits()
        with server.gui.add_folder(postural_task.getTaskID()):
            for dof in self.dof_list:
                dof_id = self.model.getVIndexFromVName(dof)
                lower = lowers[dof_id]
                upper = uppers[dof_id]

                #print(f"{dof}: ({lower}, {upper}), value: {self.initial_config[self.model.getQIndexFromQName(dof)]},    JIndex: {self.model.getJointId(dof)},   QIndex: {self.model.getQIndexFromQName(dof)},   VIndex: {self.model.getVIndexFromVName(dof)}")

                slider = server.gui.add_slider(
                    label=dof,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=self.initial_config[self.model.getQIndexFromQName(dof)])

                slider.on_update(self.on_slider_change())

                self.slider_handles.append(slider)

            reset_button = server.gui.add_button("Reset")

            @reset_button.on_click
            def _(_):
                self.postural_task.setReference(self.initial_config)

    def on_slider_change(self):
        def _(_):
            with self.lock:
                slider_ref = np.asarray([s.value for s in self.slider_handles], dtype=float)
                q_ref, _ = self.postural_task.getReference()
                i = 0
                for dof in self.dof_list:
                    q_ref[self.model.getQIndexFromQName(dof)] = slider_ref[i]
                    i += 1
                self.postural_task.setReference(q_ref)

        return _
