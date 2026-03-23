import time
import numpy as np
import argparse
import pyopensot as pysot
import h5py
from plot import joint_plot
from visualization import rvizer
from interactive_marker import interactive_marker
from solvers_sliders import iHQP_sliders


def play_robot_log(MAT_FILE, URDF_PATH, Q_VAR_NAME="q", V_VAR_NAME="v", FPS=30):
    """
    Play a robot trajectory from a .mat log in the browser using Viser.

    Parameters
    ----------
    MAT_FILE : str
        Path to the .mat file containing joint trajectories.
    URDF : robot URDF
    VAR_NAME : str, optional
        Variable name in the .mat file containing joint arrays. Default 'q_list'
    FPS : int, optional
        Frames per second for playback. Default 30
    """
    # ---- Load .mat file ----
    with h5py.File(MAT_FILE, "r") as f:
        if Q_VAR_NAME not in f:
            raise ValueError(f"Variable '{Q_VAR_NAME}' not found in {MAT_FILE}")
        q_list = np.array(f[Q_VAR_NAME])

        v_list = []
        if V_VAR_NAME not in f:
            print(f"Variable '{V_VAR_NAME}' not found in {MAT_FILE}")
        else:
            v_list = np.array(f[V_VAR_NAME])


    num_frames, dof = q_list.shape
    print(f"Loaded {num_frames} frames, DOF = {dof}")

    # ---- Start Viser server ----
    rviz = rvizer(URDF_PATH)

    # ---- GUI widgets (manual polling) ----
    gui = rviz.server.gui
    slider = gui.add_slider("/Timeline", min=0.0, max=1.0, step=1.0/num_frames, initial_value=0.0)
    checkbox = gui.add_checkbox("/Play", initial_value=True)

    # ---- Animation loop ----
    frame_dt = 1.0 / FPS
    current_frame = 0
    current_frame_float = 0.0

    with rviz.server.gui.add_folder("Positions"):
        q_plot = joint_plot(title="", size=dof, legend_label="q", server=rviz.server, dt=frame_dt)

    v_plot = None
    if len(v_list) > 0:
        with rviz.server.gui.add_folder("Velocities"):
            num_frames, vdof = v_list.shape
            v_plot = joint_plot(title="", size=vdof, legend_label="v", server=rviz.server, dt=frame_dt)

    update_plots = False # this is used only when play is false!
    while True:
        play = checkbox.value
        slider_value = slider.value

        # Only overwrite if user moved the slider
        slider_frame = round(slider_value * (num_frames - 1))
        if slider_frame != int(current_frame_float):
            update_plots = True
            current_frame_float = slider_frame

        current_frame = int(current_frame_float)

        # Update robot
        q = q_list[current_frame]
        v = None
        if len(v_list) > 0:
            v = v_list[current_frame]
        rviz.viser_urdf.update_cfg(np.array(q))

        # Advance if playing
        if play:
            current_frame_float += 1
            if current_frame_float >= num_frames:
                current_frame_float = 0.0

            # Update slider position
            slider.value = current_frame_float / (num_frames - 1)

            # Update plot
            q_plot.update(q)
            if v_plot:
                v_plot.update(v)
        else:
            if update_plots:
                q_plot.update(q)
                if v_plot:
                    v_plot.update(v)
                update_plots = False

        time.sleep(frame_dt)


def main():
    parser = argparse.ArgumentParser(
        description="Play a robot trajectory from a .mat log in the browser using Viser.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--mat_file", type=str, required=True,
                        help="Path to the .mat file with configuration data")
    parser.add_argument("--urdf_file", type=str, required=True,
                        help="Name of URDF file, e.g., panda.urdf")
    parser.add_argument("--q_var_name", type=str, default="q",
                        help="Variable name in .mat file containing position arrays")
    parser.add_argument("--v_var_name", type=str, default="v",
                        help="Variable name in .mat file containing velocity arrays")
    parser.add_argument("--fps", type=int, default=30,
                        help="Frames per second for playback")

    args = parser.parse_args()

    urdf_path = pysot.find(args.urdf_file)
    if not urdf_path:
        print(f"{args.urdf_file} not found, exiting.")
    else:
        # Call helper function
        play_robot_log(MAT_FILE=args.mat_file,
                   URDF_PATH=urdf_path,
                   Q_VAR_NAME=args.q_var_name,
                   V_VAR_NAME=args.v_var_name,
                   FPS=args.fps)

# ---------------- Command-line interface ----------------
if __name__ == "__main__":
    main()
