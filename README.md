# Dual Robot Pick-and-Place

A PyBullet-based research sandbox for studying dual-arm manipulation with two UR5 manipulators and Robotiq 85 grippers. The project bundles:

- A goal-conditioned pick-and-place environment (`PickPlace_UR5Env`) with configurable control modes, GUI support, and camera utilities.
- Low-level kinematics/dynamics wrappers for the dual-arm URDF (`UR5Robotiq85`).
- Scripts for collecting heuristic expert demonstrations and replay buffers that support Hindsight Experience Replay (HER).
- Multiple learning pipelines: weighted goal-conditioned behavior cloning (WGCSL), adversarial imitation (GAIL/WGAN-GP), and PPO fine-tuning.

The code is organized so you can generate demonstrations, train policies, evaluate saved models, and iterate on new algorithms using a consistent task setup.

## Repository Layout

| Path | Description |
| --- | --- |
| `pick_place_env.py` | Gymnasium-style environment for the dual-arm pick-and-place task, including observation/action spaces and reward logic. |
| `ur5_robotiq.py` | Robot wrapper that loads the dual UR5 URDF, maintains joint mappings, and exposes end-effector/joint-space controllers. |
| `utilize.py` | Shared helpers for connecting to PyBullet, camera utilities, scheduling, and distance computations. |
| `assets/` | URDF files for the robot, cubes, plane, and RealSense camera descriptions. |
| `get_expert_data_pick_place.py` | Collects heuristic expert trajectories and stores them as `ReplayBuffer_Trajectory` pickles. |
| `imitation_learning/` | WGCSL, GAIL, and WGAN implementations with corresponding training/evaluation scripts. |
| `reinforcement_learning/` | PPO agent and training harness tailored to the pick-and-place task. |
| `rl_utils.py` | Generic replay buffer and helper routines shared across algorithms. |
| `test.py` | Minimal script to spin up the environment for manual inspection or debugging. |
| `Dual_robot_real/` | Catkin workspace with ROS packages for running the dual-UR5 setup, RealSense cameras, and Robotiq grippers on physical hardware. |

## Getting Started

### Prerequisites

- Python 3.8+ (tested on 3.8–3.10)
- [PyBullet](https://pybullet.org/wordpress/)
- [Gymnasium](https://gymnasium.farama.org/) (for `spaces.Box` utilities)
- PyTorch (CUDA or MPS builds recommended if available)
- NumPy, tqdm, matplotlib
- Optional: Hydra, termcolor (for DP3 evaluation utilities), OpenCV, Pillow, torchvision (real-robot vision stack)
- Optional (real hardware): ROS Noetic, `ur_robot_driver`, `robotiq_2f_gripper_control`, `realsense2_camera`

### Environment Setup

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install --upgrade pip
pip install pybullet gymnasium numpy torch tqdm matplotlib
# optional extras used by scripts
pip install opencv-python pillow torchvision hydra-core termcolor dill
```

> **Tip:** When running on headless servers, disable the GUI (`sim_params['use_gui']=False`) and ensure EGL/OSMesa rendering is available if you need off-screen camera capture.

### Quick Test

To bring up the environment with the GUI and step a no-op policy:

```bash
python test.py
```

Close the window or interrupt the script to terminate.

### Simulation Configuration & Customization

- The pick-and-place environment is parameterized via three dictionaries seen across scripts:
  - `sim_params`: physics timestep (default `1/240`), GUI flag, control mode (`'end'` vs `'joint'`), training mode (controls stochastic goal sampling), and success distance threshold.
  - `robot_params`: 12-element joint reset pose (left + right arm) and gripper opening range.
  - `visual_sensor_params`: virtual camera FOV, pose, image size, and near/far planes—used by `utilize.Camera` if you enable RGB-D capture.
- Override these dictionaries before instantiating `PickPlace_UR5Env` to tailor workspace bounds, step horizons, or control schemes.
- For deterministic evaluation runs, set `sim_params['is_train']=False` to use fixed start/goal poses and disable random goal resampling.
- Logs and saved models default to the repository root. You can redirect artifacts by exporting `MODEL_DIR=/path/to/save` (scripts honor this environment variable when present).

## Core Components

### Pick-and-Place Environment

- Loads a plane, a dual UR5 robot with Robotiq 85 grippers, and two cubes (movable block + fixed goal).
- Supports `end` (end-effector) and `joint` control modes; action vectors include translational deltas and individual gripper commands.
- Observations include robot joint/EE state, achieved/desired goals, and relative offsets used for goal-conditioned learning.
- Rewards are sparse: reaching within `distance_threshold` yields success (0 reward) while otherwise returning -1, mirroring HER-friendly tasks.

### Robot Wrapper (`UR5Robotiq85`)

- Handles URDF loading, joint indexing, rest pose resets, and synchronized finger motion.
- Converts `end`-space delta commands into joint targets via PyBullet inverse kinematics.
- Applies individual scaling factors for arm movements and gripper opening to maintain stable control.

### Assets

| File | Notes |
| --- | --- |
| `assets/urdf/dual_robot.urdf` | Dual UR5 arms mounted on a shared base with paired Robotiq grippers. |
| `assets/urdf/cube_small_pick.urdf` | Movable block used as the object to pick. |
| `assets/urdf/cube_small_target_pick.urdf` | Static target marker (collision disabled with the block). |
| `assets/plane/plane.urdf` | Ground plane for the workspace. |

## Workflow Guide

### 1. Collect Expert Demonstrations (optional but recommended)

```bash
python get_expert_data_pick_place.py
```

- Launches the environment (GUI on by default) and executes a heuristic policy that actively guides the “closer” gripper to the block, closes, then lifts and moves toward the sampled goal.
- Successful trajectories accumulate in a `ReplayBuffer_Trajectory` and, once the quota is met (`expert_data_num`, default 40 000 episodes), the buffer is serialized to `dual_robot_pickplace_<N>_expert_data_WGCSL.pkl`.
- If you want a smaller pilot dataset, reduce `expert_data_num` and increase `acc_factor` thresholds; for harder randomizations you can adjust the workspace bounds inside `_sample_goal` and `_sample_achieved_goal_initial`.
- Set `sim_params['use_gui']=False` for faster headless collection—progress will still stream to stdout.

### 2. Train via WGCSL (Weighted Goal-Conditioned Supervised Learning)

```bash
python imitation_learning/tain_WGCSL.py
```

- Loads expert trajectories (if `use_expert_data=True`) and augments them with HER. When `use_expert_data=False`, the buffer is filled online using freshly collected rollouts.
- Core CLI toggles:
  - `use_expert_data`: switch between offline demonstration training vs. online collection.
  - `load_agent`: resume from stored actor/critic weights.
  - `sim_params['use_gui']`: enable/disable renderer during training and evaluation.
- Checkpoints are written to `model/wgcsl_her_dual_robot_pick_{actor,critic}_<iter>.pkl`. Set `MODEL_DIR` to change the target directory.
- After each iteration, the script constructs a non-randomized test environment (`sim_params['is_train']=False`) to report success rate, percentile statistics, and HER buffer occupancy.

### 3. Adversarial Imitation (GAIL / WGAN-GP)

```bash
python imitation_learning/train_GAIL.py
# or
python imitation_learning/train_WGAN.py
```

- Both scripts bootstrap a PPO actor/critic, sample agent rollouts, and use expert trajectories to supervise a discriminator (binary cross-entropy for GAIL, Wasserstein distance with gradient penalty for WGAN-GP).
- Key script flags:
  - `sim_params['use_gui']`: turn on visual monitoring during evaluation sweeps.
  - `her_buffer.batch_size`: dynamically overridden to match episode length; you can clamp it to stabilize critic updates.
  - `agent_num`: load a pre-trained WGCSL initialization before adversarial updates.
- Discriminator outputs become dense rewards fed back into PPO. Training logs include discriminator loss, synthetic reward magnitude, and success counters for quick diagnostics.
- Checkpoints are saved under `model/gail_dual_robot_pick_{actor,critic}_<iter>.pkl` and `model/wgan_dual_robot_pick_{actor,critic}_<iter>.pkl`; both scripts also run periodic evaluation rollouts for sanity checks.

### 4. Reinforcement Learning Fine-Tuning (PPO)

```bash
python reinforcement_learning/train_ppo.py
```

- Initializes PPO and optionally seeds it with a pre-trained WGCSL actor/critic (`agent.actor.load_state_dict(...)` / `agent.critic.load_state_dict(...)` at the top of the script).
- Collects trajectories directly from the environment, applies a HER-style goal relabeling (`ppo.her_process`), and performs clipped-surface PPO updates with entropy regularization.
- CLI knobs include entropy coefficient (`entropy_coef`), clipping range (`eps`), and number of epochs per iteration. Adjust these to trade-off stability vs. exploration.
- Actor checkpoints are stored under `model/ppo_dual_robot_pick_actor_<iter>.pkl`; critics can be snapshotted by extending the script with `torch.save(agent.critic.state_dict(), ...)`.

### 5. Evaluate / Visualize Policies

```bash
python imitation_learning/inference_wgcsl.py
```

- Example script that loads a specific WGCSL checkpoint and rolls it out (GUI enabled).
- You can adapt it for other algorithms by swapping the model class and checkpoint path.

## Artifacts & Logging

- **Model directory:** defaults to `./model`. Override via `MODEL_DIR` env var.
- **Expert buffers:** serialized to the working directory (e.g., `dual_robot_pickplace_40000_expert_data_WGCSL.pkl`). Ensure adequate disk space (~3–4 GB for 40k trajectories).
- **Progress bars:** all training scripts use `tqdm`; disable by setting `TQDM_DISABLE=1`.
- **Matplotlib plots:** WGCSL/GAIL/WGAN scripts generate learning curves at the end of execution. Uncomment `plt.show()` sections or replace with `plt.savefig(...)` to collect figures during headless runs.
- **Debug frames:** enable `sim_params['use_gui']=True` to manually inspect contact issues; combine with `utilize.Camera` for custom RGB-D recordings.

## Real-World Workspace (`Dual_robot_real/`)

The `Dual_robot_real` directory is a Catkin workspace geared toward deploying the dual-arm setup on actual UR5e manipulators with Robotiq 2F grippers and Intel RealSense cameras.

### Package Overview

- `dual_description/`, `dual_gazebo/`: URDF/xacro models, MoveIt descriptions, and Gazebo resources mirroring the physical workstation. Use these to validate collision geometry or simulate dual-arm trajectories before touching hardware.
- `dual_robot_rl/`: ROS interfaces and messages for streaming multi-step action commands; consumed by `RealDoorRunner` and other reinforcement-learning nodes.
- `dual_ur5e_driver/`, `ur_robot_driver/`: ROS control stacks for left/right UR5e arms (mirroring Universal Robots’ official driver layout) plus helper launch files that namespace each arm under `/left/` and `/right/`.
- `robotiq_driver/`: Nodes that expose Modbus control of the Robotiq 85 grippers and publish state feedback (`Robotiq2FGripperRobotInput`).
- `visual_realsense/`: Launch files, calibration YAMLs, and tf publishers for the top-mounted and right-mounted Intel RealSense cameras.
- `real_robot/`: Core research package containing Python scripts for data collection, policy execution, door-opening experiments, and vision logging.
- `move_demo/`: Example launch files and scripts for scripted joint-space trajectories—handy for sanity checks during bringup.
- `lib/`: Vendored third-party libraries (e.g., TRAC-IK bindings) compiled as part of the Catkin workspace.

### Building and Launching

```bash
cd Dual_robot_real
catkin_make
source devel/setup.bash  # or setup.zsh
```

> **Hardware prerequisites:** Ubuntu 20.04 + ROS Noetic, two UR5e arms reachable via Ethernet, Robotiq 2F grippers, and calibrated RealSense D435 cameras publishing on `/top_camera/*` and `/right_camera/*`. Ensure the UR control boxes expose the standard dashboard services (power, brake release, play/stop).

Recommended bring-up checklist:

1. Assign static IPs to each UR controller and confirm dashboard services respond:
   `rosservice call /right/ur_hardware_interface/dashboard/power_on`.
2. Launch the Robotiq driver and verify `Robotiq2FGripperRobotInput` publishes changing values while jogging the gripper from the teach pendant.
3. Start the RealSense launch files in `visual_realsense` and visualize `/top_camera/color/image_raw` and `/right_camera/aligned_depth_to_color/image_raw` in RViz; adjust exposure or ROI if needed.
4. Run `rosrun real_robot camera_test.py` to confirm the `Camera` wrapper can access both RGB and depth streams from ROS.
5. Initialize each arm using `rosrun real_robot real_robot.py` or `real_robot_right.py`, ensuring the joints move to the default `right_init_q`/`left_init_q` without collisions.

### real\_robot Package Highlights

- `scripts/utils/real_arm.py` wraps UR dashboard services, exposes flexible joint/Cartesian controllers, and relies on TRAC-IK for inverse kinematics.
- `scripts/gripper_2f.py` publishes Robotiq command messages and reads gripper state for closed-loop control.
- `scripts/vision_camera.py` subscribes to RealSense topics and returns synchronized RGB-D frames for downstream policies.
- `scripts/real_robot.py` executes dual-arm resets and Cartesian delta commands, useful for synchronized pick-and-place or teleoperation.
- `scripts/real_robot_right.py` focuses on the right arm + gripper, exporting `Robot_right`—the environment used for door-opening experiments and real-world policy evaluation.
- `scripts/multistep_wrapper.py` stacks RGB-D frames and proprioception into temporal batches, matching the input contract of the DP3 policy.
- `scripts/real_door_runner.py` couples the wrapper with learned policies, executing multi-step actions and automatically stopping the driver when episodes finish.
- `scripts/gen_demo.py` and `data_demo/*.py` encode PID-controlled waypoints for button pressing, handle grasping, and door opening, enabling reproducible human-designed demonstrations.
- `scripts/checkpoints/` houses trained DP3 weights (`latest.ckpt`, `epoch=xxxx-...ckpt`) that `scripts/eval.py` can replay through the runner. The Hydra config expected by `eval.py` lives under `scripts/policy/config` (ensure it is present in your workspace before running).

### Typical Real-Robot Workflow

1. **Bringup:** Launch UR drivers, gripper nodes, and RealSense streams (see package-specific launch files).
2. **Calibrate/Reset:** Use `real_robot.py` or `real_robot_right.py` to send the arms to safe default joint positions and verify camera feeds; tune `workspace_low/high` if your table layout differs.
3. **Safety Sweep:** Jog small deltas through `Robot_right.step` and confirm the PID gains respect joint velocity limits. Reduce `action_factor` if motion is jerky.
4. **Demonstrations:** Execute `gen_demo.py` (or `gen_demo1.py`) to follow PID-guided waypoints (`data_demo/*.py`), logging observation/action sequences and optionally recording RGB videos with `gen_video.py`.
5. **Policy Evaluation:** Install the `policy` package (`pip install -e scripts`) and run `rosrun real_robot eval.py` (Hydra config + checkpoint path configurable) to deploy the DP3 policy through `RealDoorRunner`. `MultiStepWrapper` handles frame stacking and automatically stops dashboard services once an episode terminates.
6. **Data Export:** Use `vision_data_collection.py` or `obs_gen_action.py` to capture raw RGB-D clips or convert saved observations into executable action scripts.

Troubleshooting tips:

- No IK solution: confirm `<namespace>/robot_description` publishes the URDF with the correct tool flange; TRAC-IK can fail if the camera or gripper meshes are missing.
- Speed limit warnings: reduce `arm_action_factor` or increase `action_delta_t` in `real_robot_right.py` to respect UR5 joint velocity limits.
- Depth images are 8-bit by default; convert to metric depth before feeding them to learning pipelines if required.
- Dashboard calls can hang if the controller is already playing. Issue `/.../stop` before `/.../play`.

> **Note:** The policy code referenced by `eval.py` (`policy.dp3`, Hydra configs) is not bundled in this repository. Clone or vendor the required modules before running the real-robot evaluation script. The `setup.py` in `scripts/` expects a `policy/` Python package when installing in editable mode.

## Configuration Reference

All scripts share common dictionaries for reproducibility:

- `sim_params`: GUI toggle, PyBullet timestep, control mode (`'end'` or `'joint'`), training flag (affects goal sampling), and success threshold.
- `robot_params`: Default joint angles for both arms and allowed gripper opening range.
- `visual_sensor_params`: Virtual camera intrinsics/extrinsics; used by the optional `Camera` helper for RGB-D capture or pixel-to-world projection.

Feel free to tailor these dictionaries within each script or refactor them into a central config module if you plan to sweep multiple experimental settings.

## Tips & Extensions

- **Headless training:** set `sim_params['use_gui'] = False` for faster batch runs.
- **Custom rewards or tasks:** extend `PickPlace_UR5Env` to add shaped rewards, different goal sampling, or multi-object manipulation.
- **New algorithms:** re-use the environment and replay buffers to prototype alternative RL/IL methods; both Gymnasium-style observations and normalized variants (`dictobs2npobs`) are available.
- **Rendering / cameras:** instantiate `utilize.Camera` for custom viewpoints or depth processing pipelines.
- **Sim-to-real transfer:** export the normalized observation vector alongside raw state dictionaries so you can plug the same policy into `Robot_right` with minimal feature engineering. Align workspace bounds and action scaling between `PickPlace_UR5Env` and `real_robot_right.py` to reduce distributional shift.
- **Batch experiments:** wrap training scripts with shell loops (or Hydra) to sweep seeds, control modes, or reward thresholds. All hyperparameters are regular Python literals at the top of each script for easy templating.

## License

No explicit license is provided. Please contact the repository owner or add a license file before redistributing or using the code in other projects.
