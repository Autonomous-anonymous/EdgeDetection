## Starting the project on Windows

The Gazebo simulation environment is run inside WSL Ubuntu 24.04.

Open **Miniforge Prompt** (or Windows Command Prompt) and start Ubuntu:

```bat
wsl -d Ubuntu-24.04
```

Move to the project directory:

```bash
cd ~/edge-path-project-git
```

Set the Gazebo model resource path:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/edge-path-project-git/edge-path-project/gazebo/models:$GZ_SIM_RESOURCE_PATH"
```

The project is now ready to use.

For example, start the two-vehicle Gazebo world with:

```bash
gz sim \
edge-path-project/gazebo/worlds/ruskotunturi_final_sil_demo_world.sdf
```

Press **Play** in Gazebo before starting a test script.

Open another Miniforge Prompt / terminal, enter WSL again:

```bat
wsl -d Ubuntu-24.04
```

Then return to the project directory:

```bash
cd ~/edge-path-project-git
```

For example, run the two-vehicle merge test:

```bash
python3 \
edge-path-project/gazebo/scripts/ruskotunturi_final_sil_demo.py
```

> **Note:** No Conda environment activation is currently required. Gazebo and the required Python Gazebo bindings are installed inside WSL Ubuntu.
