from scipy.spatial.transform import Rotation as R
from xbot2_interface import Affine3
import numpy as np

class com_marker():
    def __init__(self, server, com_task, lock, slider_max=1., slider_step=0.01):
        self.com_task = com_task
        self.lock = lock

        p, _, *_ = self.com_task.getReference()

        self.int_marker = server.scene.add_transform_controls(
            "/"+com_task.getTaskID(),
            position=p,
            disable_rotations=True,
            scale=0.4
        )
        self.int_marker.on_update(self.on_target_move())

        with server.gui.add_folder(com_task.getTaskID()):
            self.lambda_slider = server.gui.add_slider(label="lambda", min=0., max=slider_max, step=slider_step, initial_value=com_task.getLambda())
            self.lambda_slider.on_update(self.on_slider_change())

            self.is_active_checkbox = server.gui.add_checkbox("/Enable", initial_value=True)
            self.is_active_checkbox.on_update(self.on_checkbox_change())


    def on_target_move(self):
        def _(_):
            p = self.int_marker.position

            new_p = np.array(p)

            with self.lock:
                self.com_task.setReference(new_p)
        return _

    def on_slider_change(self):
        def _(_):
            with self.lock:
                self.com_task.setLambda(self.lambda_slider.value)
        return _

    def on_checkbox_change(self):
        def _(_):
            is_active = self.is_active_checkbox.value
            if is_active: # switch to active
                p = self.com_task.getActualPose()
                self.int_marker.position = (p[0], p[1], p[2])
            self.int_marker.visible = is_active
        return _


class interactive_marker():

    def __init__(self, server, cartesian_task, lock, slider_max=1., slider_step=0.01):

        self.cartesian_task = cartesian_task
        self.lock = lock

        T, _, *_ = self.cartesian_task.getReference()
        quat = R.from_matrix(T.linear).as_quat()  # x y z w

        self.int_marker = server.scene.add_transform_controls(
            "/"+cartesian_task.getTaskID(),
            position=(T.translation[0], T.translation[1], T.translation[2]),
            wxyz=(quat[3], quat[0], quat[1], quat[2]),
            scale=0.4
        )
        self.int_marker.on_update(self.on_target_move())

        with server.gui.add_folder(cartesian_task.getTaskID()):
            self.lambda_slider = server.gui.add_slider(label="lambda", min=0., max=slider_max, step=slider_step, initial_value=cartesian_task.getLambda())
            self.lambda_slider.on_update(self.on_slider_change())

            self.is_active_checkbox = server.gui.add_checkbox("/Enable", initial_value=True)
            self.is_active_checkbox.on_update(self.on_checkbox_change())


    def on_target_move(self):
        def _(_):
            p = self.int_marker.position
            o = self.int_marker.wxyz

            new_T = Affine3()
            new_T.translation = np.array(p)

            r = R.from_quat([o[1], o[2], o[3], o[0]])
            new_T.linear = r.as_matrix()

            with self.lock:
                self.cartesian_task.setReference(new_T)
        return _

    def on_slider_change(self):
        def _(_):
            with self.lock:
                self.cartesian_task.setLambda(self.lambda_slider.value)
        return _

    def on_checkbox_change(self):
        def _(_):
            is_active = self.is_active_checkbox.value
            if is_active: # switch to active
                T = self.cartesian_task.getActualPose()
                quat = R.from_matrix(T.linear).as_quat()  # x y z w
                self.int_marker.position = (T.translation[0], T.translation[1], T.translation[2])
                self.int_marker.wxyz = (quat[3], quat[0], quat[1], quat[2])
            self.int_marker.visible = is_active
        return _
