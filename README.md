# Dual Robot Manipulation Sandbox

A research playground for learning-based manipulation with UR5 manipulators in PyBullet. The repo now ships three goal-conditioned tasks that share a consistent observation/action API:

- **Dual-arm pick-and-place** – legacy task with two UR5 arms and Robotiq 85 grippers (`PickPlace_UR5Env`).
- **Single-arm UR5e pick-and-place** – lighter-weight task that mirrors the dual-arm goal structure (`PickPlaceUR5eEnv`).
- **Single-arm UR5e reach** – reaching the handle with sparse reward shaped for HER (`ReachUR5Env`).

On top of these environments you will find heuristic data collection policies, imitation-learning pipelines (WGCSL, GAIL, WGAN-GP), and several RL baselines (PPO, SAC+HER, DDPG+HER). The code is organised so that demonstrations, training runs, evaluation scripts, and real-robot deployments can reuse the same environment definitions.

---

## Requirements

- Python 3.9+ (3.8–3.11 tested)
- A GPU is recommended for faster training but not required (CUDA or Metal backends work out of the box).
- Install Python dependencies with:

```bash
python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

> **Tip:** When running on a headless server set `--no-gui` when using `test.py` or toggle `sim_params['use_gui']=False` in scripts.

---

## Repository Layout

| Path | Description |
| --- | --- |
| `pick_place_env.py` | Dual-arm and single-arm pick-and-place environments (Gymnasium-style API). |
| `reach_env.py` | UR5e reach environment built on the same goal-conditioned interface. |
| `ur5_robotiq.py` | Robot wrappers for dual UR5+Robotiq85 and single UR5e+Robotiq140 embeddings. |
| `assets/` | URDFs, meshes, and environment objects used by the simulators. |
| `get_expert_data_pick_place.py` | Heuristic data collection for the dual-arm pick-and-place task (HER-ready trajectories). |
| `get_expert_data_reach.py` | Heuristic reach data collection for the UR5e environment. |
| `imitation_learning/` | WGCSL, GAIL, WGAN-GP implementations and training scripts. |
| `reinforcement_learning/` | PPO, SAC+HER, DDPG+HER agents, training harnesses, and inference utilities. |
| `Dual_robot_real/` | Catkin workspace for the physical dual-UR5 setup (ROS Noetic). |
| `test.py` | CLI utility to instantiate any of the environments for smoke testing. |

---

## Quick Sanity Check

Bring up one of the tasks with a zero-action policy to validate your install:

```bash
python test.py --robot-type dual        # dual-arm pick-and-place
python test.py --robot-type single      # single-arm UR5e pick-and-place
python test.py --robot-type reach       # single-arm reach task
```

Pass `--no-gui` for headless runs and `--train-mode` to enable stochastic goal sampling.

---

## Collecting Demonstrations

### Dual-arm pick-and-place
```bash
python get_expert_data_pick_place.py
```
The script records successful heuristic rollouts, augments them with HER, and stores a `ReplayBuffer_Trajectory` pickle once the target count is reached.

### UR5e reach
```bash
python get_expert_data_reach.py
```
Generates sparse-reward trajectories for the reach task and serialises them to `ur5_reach_<N>_expert_data.pkl`.

Both collectors expose `sim_params`, `robot_params`, and `visual_sensor_params` dictionaries at the top of the script—tune these to change workspace bounds, GUI usage, or camera placement.

---

## Training Pipelines

### Reinforcement Learning

```bash
# SAC + HER on the UR5e pick-and-place task
python reinforcement_learning/train_sac.py

# DDPG + HER on the UR5e pick-and-place task
python reinforcement_learning/train_ddpg.py

# PPO fine-tuning on the dual-arm task
python reinforcement_learning/train_ppo.py
```

Each training script supports HER replay buffers, learning-rate decay, and periodic evaluation. Saved models land in the `model/` directory by default.

### Imitation Learning

```bash
# Weighted Goal-Conditioned Supervised Learning
python imitation_learning/tain_WGCSL.py

# GAIL / WGAN-GP adversarial imitation
python imitation_learning/train_GAIL.py
python imitation_learning/train_WGAN.py
```

Set `use_expert_data` toggles inside the scripts to switch between offline demonstration training and online data collection. All scripts share the goal-conditioned observation format emitted by the environments.

---

## Evaluating Policies

```bash
# Run a trained SAC policy (update the checkpoint index/path as needed)
python reinforcement_learning/inference_sac.py

# Run a trained DDPG policy
python reinforcement_learning/inference_ddpg.py

# Visualise a WGCSL actor
python imitation_learning/inference_wgcsl.py
```

Adjust `model_path` / `model_num` arguments inside each script to point to the checkpoint you want to evaluate.

---

## Working With the Real Robot

The `Dual_robot_real/` directory contains a Catkin workspace with launch files and ROS packages for the physical dual-UR5 rig, RealSense cameras, and Robotiq grippers. Simulation and learning scripts in this repository remain decoupled, so you can iterate in PyBullet and deploy with your preferred ROS bridge.

---

## Troubleshooting & Tips

- **Slow simulation:** Drop `sim_params['n_sub_step']` or disable the GUI. PyBullet GUI often limits the framerate—headless runs are considerably faster.
- **GPU selection:** Scripts automatically prefer CUDA, falling back to Metal (MPS) or CPU. Override by setting `CUDA_VISIBLE_DEVICES=""` or editing the device selection block.
- **Asset paths:** All URDF and mesh paths are relative to the repo root (`./assets/...`). If you embed these modules elsewhere, preserve the directory structure or update the path strings.
- **Legacy folder:** The older `UR5e_with_RL/` snapshot has been merged into the main codebase and can be removed after verifying no custom scripts depend on it.

Happy experimenting!
