<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# intball2_common

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#overview">Overview</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
      <ul>
        <li><a href="#requirements">Requirements</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#milestones">Milestones</a></li>
  </ol>
</details>

## Overview
Packages for controlling the robot in the Int-Ball2 simulator.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Setup
This section explains how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Requirements
Set up the following environment before proceeding to the installation steps below.
| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1. Move to the `src` folder of your ROS workspace.
   ```sh
   cd　~/colcon_ws/src/
   ```
2. Clone this repository.
   ```sh
   git clone https://github.com/TeamSOBITS/intball2_common.git
   ```
3. Move into the repository.
   ```sh
   cd intball2_common
   ```
4. Install dependencies.
    ```sh
    bash install.sh
    ```
5. Build the packages.
   ```sh
   cd ~/colcon_ws/
   colcon_build
   source ~/colcon_ws/devel/setup.bash
   ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## TF Tree

```sh
base
├── iss_body
│  └── dock_body
└── body
    ├── cameraF_link
    ├── cameraL_link
    ├── cameraR_link
    └── imu_link
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Usage
First, launch the Int-Ball2 simulator.
Then, launch the ROS2 Bridge container.

### Displaying the ISS model in Rviz
```sh
ros2 launch intball2_programs iss_model.launch.py 
```

### Spawning obstacles
1. Broadcast the TF frames used to place obstacles. Placement locations are defined in [spawn_locations.yaml](intball2_programs/locations/spawn_locations.yaml).
    ```sh
    ros2 run intball2_programs spawn_location_broadcaster
    ```
2. Spawn an obstacle at a position relative to the ISS in Gazebo.
    ```sh
    ros2 run intball2_programs spawn_model [args]
    ```

#### Arguments

| Argument | Description | Default |
| --- | --- | --- |
| `-m` | Model name (see "Available models" below) | `box_obstacle` |
| `-n` | Model instance name (auto-generated if omitted) | `{model}_{uuid}` |
| `-f` | Reference frame | `iss_body` |
| `-s` | Size in meters (only applies to `box`-type models) | `0.45 0.25 1.7` |
| `-o` | Offset in meters | `0.0 0.0 0.0` |
| `-r` | Rotation in degrees | `0.0 0.0 0.0` |
| `-c` | Enable collision flag | disabled |

#### Available models

| Value for `-m` | Description |
| --- | --- |
| `box` (default: `box_obstacle`) | Rectangular box. Size controlled by `-s`, collision by `-c` |
| `human` | Standing human model on the ground. Also floats since `gravity=0` |
| `laptop` | Laptop model from Gazebo Fuel |
| `float_blue`, `float_orange`, `float_purple` | Floating human model (crouched floating pose, different colors) |
| `float2_blue`, `float2_green`, `float2_orange`, `float2_purple` | Floating human model (standing floating pose, different colors) |
| `tape` | Competition tape (Gaffers Tape) model |
| `ctb_1023`, `ctb_1456`, `ctb_2305`, `ctb_2789`, `ctb_4678` | CTB (Cargo Transfer Bag) models (different part numbers) |
| `astrobee_freeflyer` | Astrobee free-flyer robot model |

`float_*`/`float2_*`/`tape`/`ctb_*`/`astrobee_freeflyer` reference local meshes from the
competition simulator (Noetic side). Collision is only added when `-c` is specified
(`-s` does not apply); otherwise only a visual model is placed.

Lights can also be spawned individually via `-m`, but since they have no collision/size
concept and a dedicated batch command exists, see "Spawning lights" below for details.

#### Examples

```sh
# Basic usage (spawn with default values)
ros2 run intball2_programs spawn_model

# Spawn a box with specified size, rotation, and collision
ros2 run intball2_programs spawn_model -m box -s 0.3 0.3 0.3 -r 0.0 0.0 90.0 -c

# Change the reference frame to world and spawn a floating human model with collision
ros2 run intball2_programs spawn_model -m float_blue -f iss_body -o 10.8 -6.0 4.8 -c
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Spawning lights
Lights inside the ISS, defined in [spawn_locations.yaml](intball2_programs/locations/spawn_locations.yaml), can be turned on/off in Gazebo all at once.

- Turn all on (foreground command; keeps following the ISS's motion while running, and turns off (auto-delete) on Ctrl+C)
    ```sh
    ros2 run intball2_programs spawn_lights
    ```
- Turn on a single light (example)
    ```sh
    ros2 run intball2_programs spawn_model -m iss_light_1 -f iss_light_1
    ```

`delete_lights` is not the normal way to turn lights off; it is a manual cleanup
fallback for when `spawn_lights` terminates abnormally (e.g. `kill -9`) and lights
are left on.
```sh
ros2 run intball2_programs delete_lights
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Relative movement
Int-Ball2 can be moved using relative coordinates.
```sh
ros2 run intball2_programs move_relative [args]
```

#### Arguments

| Short | Long | Description | Default |
| --- | --- | --- | --- |
| `-x` | `--x` | Relative movement along X (meters) | `0.0` |
| `-y` | `--y` | Relative movement along Y (meters) | `0.0` |
| `-z` | `--z` | Relative movement along Z (meters) | `0.0` |
| `-r` | `--roll` | Relative rotation, roll (degrees) | `0.0` |
| `-p` | `--pitch` | Relative rotation, pitch (degrees) | `0.0` |
| `-w` | `--yaw` | Relative rotation, yaw (degrees) | `0.0` |

#### Examples

```sh
# Basic usage (move 0.5m along Z)
ros2 run intball2_programs move_relative -z 0.5

# Move along X and Y simultaneously
ros2 run intball2_programs move_relative -x 0.3 -y -0.2

# Combine movement and rotation (move 1.0m along Z while rotating 90 degrees about Z)
ros2 run intball2_programs move_relative -z 1.0 -w 90.0

# Specify roll angle
ros2 run intball2_programs move_relative -r 45.0

# Combine multiple movements
ros2 run intball2_programs move_relative -x 0.5 -y 0.5 -z 0.5 -r 0.0 -p 0.0 -w 45.0
```

### Displaying PLY files
PLY files produced by 3D Gaussian Splatting can be displayed as point cloud data.
```sh
ros2 launch intball2_programs ply_display.launch.py
```

To launch only the publisher
```sh
ros2 run intball2_programs ply_publisher 
```
```sh
# Specify the path of the PLY file inside the models directory
ply_path = os.path.join(package_share_dir, 'models', 'iss_30000.ply')
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Generating stereo point clouds
Generate a disparity image and point cloud (`/stereo/points2`) from the left/right camera images, viewable in RViz, etc.
```sh
ros2 launch intball2_programs stereo_pointcloud.launch.py
```

#### Parameters

| Parameter | Description | Default |
| --- | --- | --- |
| `left_image_topic` | Left camera image topic | `/camera_left/image_raw` |
| `right_image_topic` | Right camera image topic | `/camera_right/image_raw` |
| `left_info_topic` | Left camera camera_info topic | `/camera_left/camera_info_fixed` |
| `right_info_topic` | Right camera camera_info topic | `/camera_right/camera_info_fixed` |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Milestones

See the [Issues page](https://github.com/TeamSOBITS/intball2_common/issues) for current bugs and feature requests.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/intball2_common.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/intball2_common/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/intball2_common.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/intball2_common/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/intball2_common.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/intball2_common/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/intball2_common.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/intball2_common/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/intball2_common.svg?style=for-the-badge
[license-url]: LICENSE
