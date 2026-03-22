import time
import numpy as np
import viser
from viser.extras import ViserUrdf
import argparse
import pyopensot as pysot
import h5py
from yourdfpy import URDF
from plot import joint_plot


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
    server = viser.ViserServer(verbose=True)
    scene = server.scene

    # ---- Load URDF ----
    urdf = URDF.load(
            URDF_PATH,
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=True,
            build_collision_scene_graph=True,
        )



    base = scene.add_frame("/robot_base", show_axes=False)
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf,
        root_node_name="/robot_base",
        load_meshes=True,
        load_collision_meshes=True,
        collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
    )

    # ---- Add ground grid ----
    scene.add_grid(
        "/ground_grid",
        width=5.0,
        height=5.0,
        width_segments=20,
        height_segments=20,
    )

    # Camera setup
    # Set the initial camera pose before showing the scene
    server.initial_camera.position = (1.5, 1.5, 1.5)
    server.initial_camera.look_at = (0.0, 0.0, 0.5)
    # Optionally adjust up direction
    server.initial_camera.up = (0.0, 0.0, 1.0)

    # ---- GUI widgets (manual polling) ----
    gui = server.gui
    slider = gui.add_slider("/Timeline", min=0.0, max=1.0, step=1.0/num_frames, initial_value=0.0)
    checkbox = gui.add_checkbox("/Play", initial_value=True)

    # ---- Visualization ---- #
    # Add visibility checkboxes.
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

    # ---- Animation loop ----
    frame_dt = 1.0 / FPS
    current_frame = 0
    current_frame_float = 0.0

    with server.gui.add_folder("Positions"):
        q_plot = joint_plot(title="", size=dof, legend_label="q", server=server, dt=frame_dt)

    v_plot = None
    if len(v_list) > 0:
        with server.gui.add_folder("Velocities"):
            num_frames, vdof = v_list.shape
            v_plot = joint_plot(title="", size=vdof, legend_label="v", server=server, dt=frame_dt)

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
        viser_urdf.update_cfg(np.array(q))

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
