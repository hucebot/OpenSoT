from pyopensot.tasks.velocity import Cartesian
from scipy.spatial.transform import Rotation as R
from xbot2_interface import Affine3
import numpy as np


class interactive_marker():


    def __init__(self, server, cartesian_task, lock):

        self.cartesian_task = cartesian_task
        self.lock = lock

        T, _ = self.cartesian_task.getReference()
        quat = R.from_matrix(T.linear).as_quat()  # x y z w

        self.int_marker = server.scene.add_transform_controls(
            "/"+cartesian_task.getTaskID(),
            position=(T.translation[0], T.translation[1], T.translation[2]),
            wxyz=(quat[3], quat[0], quat[1], quat[2]),
            scale=0.4
        )

        # register callback
        self.int_marker.on_update(self.on_target_move())



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
