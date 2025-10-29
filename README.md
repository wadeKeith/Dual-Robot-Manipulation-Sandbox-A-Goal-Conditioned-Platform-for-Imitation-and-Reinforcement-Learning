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

- Python 3.8+
- [PyBullet](https://pybullet.org/wordpress/)
- [Gymnasium](https://gymnasium.farama.org/) (for `spaces.Box` utilities)
- PyTorch (tested with CUDA and MPS backends)
- NumPy, tqdm, matplotlib

### Environment Setup

```bash
python -m venv .venv
source .venv/bin/activate  # On Windows use: .venv\Scripts\activate
pip install --upgrade pip
pip install pybullet gymnasium numpy torch tqdm matplotlib
```

> **Tip:** If you plan to render the PyBullet GUI, use a local Python environment rather than headless servers.

### Quick Test

To bring up the environment with the GUI and step a no-op policy:

```bash
python test.py
```

Close the window or interrupt the script to terminate.

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
- Successful trajectories are appended to a `ReplayBuffer_Trajectory` and saved as `dual_robot_pickplace_<N>_expert_data_WGCSL.pkl`.
- Adjust `sim_params` or the heuristic in the script if you need different behavior or dataset sizes.

### 2. Train via WGCSL (Weighted Goal-Conditioned Supervised Learning)

```bash
python imitation_learning/tain_WGCSL.py
```

- Loads expert trajectories (if `use_expert_data` is `True`) and augments them with HER.
- Optimizes a stochastic policy network alongside value/Q estimators; checkpoints are written to `model/wgcsl_her_dual_robot_pick_{actor,critic}_<epoch>.pkl`.
- After each epoch, the script switches to evaluation mode (`is_train = False`) for a success-rate sanity check.

### 3. Adversarial Imitation (GAIL / WGAN-GP)

```bash
python imitation_learning/train_GAIL.py
# or
python imitation_learning/train_WGAN.py
```

- Both scripts boot a PPO policy, sample agent rollouts, and use expert trajectories to supervise a discriminator.
- Discriminator outputs become learned rewards fed back into PPO updates.
- Checkpoints are saved under `model/gail_dual_robot_pick_*.pkl` and `model/wgan_dual_robot_pick_*.pkl`.
- `train_WGAN.py` uses gradient penalty (`c_lambda`) and RMSProp optimizers to stabilize Wasserstein training.

### 4. Reinforcement Learning Fine-Tuning (PPO)

```bash
python reinforcement_learning/train_ppo.py
```

- Initializes PPO and optionally seeds it with a pre-trained WGCSL actor/critic.
- Collects trajectories directly from the environment, applies a HER-style goal relabeling (`ppo.her_process`), and performs clipped-surface PPO updates.
- Models are stored under `model/ppo_dual_robot_pick_actor_<epoch>.pkl`.

### 5. Evaluate / Visualize Policies

```bash
python imitation_learning/inference_wgcsl.py
```

- Example script that loads a specific WGCSL checkpoint and rolls it out (GUI enabled).
- You can adapt it for other algorithms by swapping the model class and checkpoint path.

## Real-World Workspace (`Dual_robot_real/`)

The `Dual_robot_real` directory is a Catkin workspace geared toward deploying the dual-arm setup on actual UR5e manipulators with Robotiq 2F grippers and Intel RealSense cameras.

### Package Overview

- `dual_description/`, `dual_gazebo/`: URDF/xacro models, MoveIt descriptions, and Gazebo resources mirroring the physical workstation.
- `dual_ur5e_driver/`, `ur_robot_driver/`: ROS control and hardware interfaces for left/right UR5e arms (mirrors Universal Robotics' official driver layout).
- `robotiq_driver/`: ROS wrappers for commanding the Robotiq 85 grippers via the Modbus bridge.
- `visual_realsense/`: Launch files and calibration assets for top/right RealSense RGB-D sensors.
- `real_robot/`: Core research package containing Python scripts for data collection, policy execution, and door-opening experiments.

### Building and Launching

```bash
cd Dual_robot_real
catkin_make
source devel/setup.bash  # or setup.zsh
```

> **Hardware prerequisites:** Ubuntu 20.04 + ROS Noetic, two UR5e arms reachable via Ethernet, Robotiq 2F grippers, and calibrated RealSense D435 cameras publishing on `/top_camera/*` and `/right_camera/*`. Ensure the UR control boxes expose the standard dashboard services (power, brake release, play/stop).

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
2. **Calibrate/Reset:** Use `real_robot.py` or `real_robot_right.py` to send the arms to safe default joint positions and verify camera feeds.
3. **Demonstrations:** Execute `gen_demo.py` (or variants) to roll out PID-guided trajectories, log observations/actions, and optionally record RGB videos.
4. **Policy Evaluation:** Install the `policy` package (`pip install -e scripts`) and run `rosrun real_robot eval.py` (Hydra config + checkpoint path configurable) to deploy the DP3 policy through `RealDoorRunner`.
5. **Data Export:** Scripts such as `vision_data_collection.py` capture raw RGB-D sequences for offline annotation or dataset curation.

> **Note:** The policy code referenced by `eval.py` (`policy.dp3`, Hydra configs) is not bundled in this repository. Clone or vendor the required modules before running the real-robot evaluation script.

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

## License

No explicit license is provided. Please contact the repository owner or add a license file before redistributing or using the code in other projects.
