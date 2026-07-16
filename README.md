# ALM

[![Build](https://github.com/MartinHaghani/ALM/actions/workflows/build-image.yaml/badge.svg)](https://github.com/MartinHaghani/ALM/actions/workflows/build-image.yaml)

ALM is a standalone autonomous-lawn-mower project and ROS Noetic workspace. It
historically descends from OpenMower and retains OpenMower-compatible ROS package,
launch, configuration, and deployment identifiers where changing them would break
existing systems. ALM's canonical repository is
[MartinHaghani/ALM](https://github.com/MartinHaghani/ALM).

There are references to other repositories (libraries) needed to build the software. This way, we can track the exact version of the packages used in each release to ensure package compatibility.
Currently, the following repositories are included:

- **slic3r_coverage_planner**: A coverage planner based on the Slic3r software for 3d printers. This is used to plan the mowing path.
- **teb_local_planner**: The local planner which allows the robot to avoid obstacles and follow the global path using kinematic constraints.
- **xesc_ros**: The ROS interface for the xESC motor controllers.

## Container images: Default vs Legacy

If your robot runs the latest OpenMower OS (v2): use the images without prefix or suffix (e.g. `latest`, `v1.2.3`).
These images only contain the OpenMower ROS stack and expect the OS to provide web and MQTT services (for example via your system’s compose setup).

If your robot runs an old version of OpenMower OS v1 (Legacy): use the legacy image.
The OS doesn't provide web and MQTT services, so the image contains nginx and mosquitto to provide these services inside the container.
The Docker images have a `-legacy` suffix or `releases-` prefix: (e.g. `releases-edge`, `v1.2.3-legacy`).

## Documentation

For repo-specific, maintained contributor and agent docs, start with:

- [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md)
- [docs/README.md](docs/README.md)
- [docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md)
- [docs/RASPBERRY_PI.md](docs/RASPBERRY_PI.md)
- [docs/CONFIGURATION.md](docs/CONFIGURATION.md)
- [CONTRIBUTING.md](CONTRIBUTING.md)

## Getting started

### Running on your machine

ALM targets ROS Noetic. ([installation instructions](http://wiki.ros.org/noetic/Installation)) There is no distributed release package yet; for development and testing, build the workspace locally.

#### Fetch Dependencies

Before building, you need to fetch this project's dependencies. The best way to do this is by using rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Run in the repository's root:

```bash

rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
```

#### Build workspace

Just build as any other ROS workspace: `catkin_make`
Once it's done, another step is to source workspace env vars:

```bash
source devel/setup.bash
```

#### Launch ALM

ALM retains the OpenMower-compatible ROS package and launch names. The workspace
contains several [roslaunch](http://wiki.ros.org/roslaunch) files in
`src/open_mower/launch`; `open_mower.launch` composes the primary runtime.

```bash
roslaunch open_mower open_mower.launch
```

Before you launch `open_mower` package, env vars with configuration have to be set.

```bash
cp config/mower_config.sh.example mower_config.sh
source mower_config.sh # it's expected to adjust the file
```

The shell example is deprecated and retained for legacy compatibility. Prefer the
structured configuration workflow documented in
[docs/CONFIGURATION.md](docs/CONFIGURATION.md). The similarly named file under
`src/open_mower/config/` is only a redirect stub.

### Running in a container

See [docs/DOCKER.md](docs/DOCKER.md) for the maintained default/legacy image split,
runtime assumptions, and development-container workflow.

## Contribution

### How to Build Using CLion IDE

First, launch CLion in a sourced environment. For this I use the following bash file:

```bash
#!/bin/zsh

source <your_absolute_path_to_repository>/devel/setup.zsh

# You can find this path in the Jetbrains Toolbox
nohup <your_absolute_path_to_clion>/clion.sh >/dev/null 2>&1 &
```


Then, open the `src` directory. CLion will prompt with the following screen:

![CLion CMake Settings](./img/clion_cmake_settings.png)

Copy the settings for **Build directory** and **CMake options**. Everything else can stay the same. This is all you need!


# Notes / ToDos

- For local navigation, I have tried to use the teb_local_planner. Unfortunately, it seems that (at least for me) the noetic version is VERY broken. Therefore I added the current melodic dev version as git submodule to this repo. It seems to work fine with ROS noetic and this setup here.
- If the map has no docking point set, planning crashes as soon as we try to approach the docking point. TODO(#14): check before even starting to mow.

# License

This work is licensed under the [GNU General Public License version 3](https://www.gnu.org/licenses/gpl-3.0.html). See the [LICENSE](LICENSE) file for details.
