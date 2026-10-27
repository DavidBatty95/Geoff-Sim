# Online tutorials, repositories and libraries

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../README.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../literature_review/README.md"><img src="../media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

[← Resource library](README.md) · [Project code map](../REPOSITORY_GUIDE.md) · [50-paper archive](RESEARCH.md)

[![Tutorials, repositories and libraries](../media/resources/simulation.svg)](#browse-the-reference-directory)

**The central external-reference directory for Geoff-Sim.** Find an online
tutorial, API reference, important repository or supporting library here, then
follow the local-context links to see how it relates to this project.
Reviewed **5 October 2026**; references are grouped by purpose rather than popularity.

## Browse the reference directory

| Topic | What you will find |
| --- | --- |
| [Suggested learning paths](#suggested-learning-paths) | Short sequences for ROS beginners, navigation, manipulation and policy development |
| [ROS 2 fundamentals](#ros-2-fundamentals-and-tutorials) | Official Humble tutorials, launch, QoS, executors and recording |
| [ROS 2 packages and standards](#ros-2-package-apis-and-standards) | Client APIs, TF, descriptions, messages, middleware and frame conventions |
| [Navigation and mapping](#navigation-and-mapping) | Nav2 walkthroughs, SLAM, behavior trees and estimator alternatives |
| [Manipulation and control](#manipulation-and-control) | MoveIt, ros2_control, motion planning and kinematics |
| [MuJoCo and Unitree](#mujoco-and-unitree) | Engine tutorials, MJCF, simulator sources and robot interfaces |
| [Learning and policy deployment](#learning-and-policy-deployment) | MJLab, Warp, PPO tooling, PyTorch and ONNX |
| [Perception and numerical libraries](#perception-geometry-and-numerical-libraries) | Images, point clouds, transforms, optimization and scientific Python |
| [Recording and diagnostics](#recording-debugging-and-profiling) | Bags, visualization, tracing and offline analysis |
| [Build tools and community](#build-test-and-community-resources) | Containers, colcon, CI, tests and learning communities |
| [Project pins and access](#project-pins-and-compatibility) | Actual checkout URLs, policy identity and known access limitations |

### How to read the labels

- **Used:** part of the documented Geoff-Sim runtime or build ecosystem.
- **Training:** relevant to the separate optional training environment.
- **Installed / reference:** available in the image, but not the owner of the
  default robot command path.
- **Optional:** useful for diagnostics or future experiments; not claimed as an
  integrated default.
- **Learning:** an explanatory resource rather than a project dependency.

Geoff-Sim targets **ROS 2 Humble**. Prefer Humble-specific examples and API pages.
Current upstream branches, rolling documentation and library `stable` pages can
describe newer interfaces. This directory is a reference list, not a dependency
lockfile: the [Docker configuration](../../docker/README.md),
[package manifests](../../ros2_ws/src/README.md) and
[patch manifest](../../patches/manifest.json) define the actual environment.

## Suggested learning paths

| Goal | Follow this sequence | Apply it locally |
| --- | --- | --- |
| Learn ROS 2 | [Official Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html) → [Python publisher/subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) → [launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) → [TF2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) | [First demo](../DEMO_GUIDE.md) and [workspace packages](../../ros2_ws/src/README.md) |
| Understand robot navigation | [Nav2 concepts](https://docs.nav2.org/rolling/getting_started/navigation_concepts/) → [robot setup guide](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/) → [configuration/tuning](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/) | [Onboard architecture](../ARCHITECTURE.md), [source boundaries](../ONBOARD_SOCIAL_NAVIGATION.md); compare with Humble source |
| Learn arm planning | [Modern Robotics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/) → [MoveIt Humble tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html) → [planning-scene concepts](https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html) | [MoveIt configuration](../../ros2_ws/src/g1_moveit_config/README.md) and [manipulation server](../../ros2_ws/src/g1_manipulation/README.md) |
| Explore physics and policies | [MuJoCo Python tutorial](https://mujoco.readthedocs.io/en/stable/python.html#tutorial-notebook) → [MJLab docs](https://mujocolab.github.io/mjlab/) → [PyTorch tutorials](https://docs.pytorch.org/tutorials/) | [Training/calibration](../TRAINING_AND_CALIBRATION.md) and [payload experiment](../PAYLOAD_POLICY_TRAINING.md) |
| Diagnose a slow or unstable run | [QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) → [executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) → [rosbag2](https://github.com/ros2/rosbag2) → [ROS 2 tracing](https://github.com/ros2/ros2_tracing) | [Results review](../RESULTS_REVIEW.md), [response tools](../../scripts/README.md) and [development checks](../DEVELOPMENT.md) |

## ROS 2 fundamentals and tutorials

| Reference | What to learn | Role |
| --- | --- | --- |
| [ROS 2 Humble documentation](https://docs.ros.org/en/humble/) | Distribution-specific concepts, tutorials and how-to guides | Used / learning |
| [Tutorial index](https://docs.ros.org/en/humble/Tutorials.html) | Progress from CLI inspection to client libraries and intermediate topics | Learning |
| [Publisher/subscriber in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) | Node creation, messages, callbacks and packaging | Learning |
| [Launch tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) | Multi-node composition, arguments and configuration | Used / learning |
| [TF2 introduction](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) | Frame trees, transforms and time-dependent lookup | Used / learning |
| [URDF tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) | Links, joints, robot descriptions and visualization | Used / learning |
| [Quality of service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | Reliability, history, durability and sensor-stream compatibility | Used / learning |
| [Executor concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) | Callback scheduling and execution behavior | Used / learning |
| [Using callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) | Concurrency, mutual exclusion and avoiding callback deadlocks | Used / learning |
| [Recording and playback](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) | Record topic data and inspect playback | Optional diagnostic workflow |
| [ROS 2 example repository](https://github.com/ros2/examples/tree/humble) | Small C++ and Python examples aligned with Humble | Learning |

ROS documentation may require browser verification when accessed by automated
tools. That challenge is different from a missing tutorial.

## ROS 2 package APIs and standards

| Package / standard | Documentation or source | Useful for |
| --- | --- | --- |
| `rclpy` | [Humble API](https://docs.ros.org/en/humble/p/rclpy/) · [source](https://github.com/ros2/rclpy/tree/humble) | Python nodes, actions, services, executors and parameters; used by bridge/autonomy |
| `rclcpp` | [Humble API](https://docs.ros.org/en/humble/p/rclcpp/) · [source](https://github.com/ros2/rclcpp/tree/humble) | C++ ROS clients used by local action servers and plugins |
| TF2 / `geometry2` | [Humble source](https://github.com/ros2/geometry2/tree/humble) | Transform buffers, listeners and frame math |
| `common_interfaces` | [Humble source](https://github.com/ros2/common_interfaces/tree/humble) | Sensor, geometry, navigation and visualization message definitions |
| `vision_msgs` | [Source](https://github.com/ros-perception/vision_msgs) | Detection and vision-message contracts; select the installed release |
| `robot_state_publisher` | [Source](https://github.com/ros/robot_state_publisher) | Convert joint states and URDF into robot transforms |
| Xacro | [Source](https://github.com/ros/xacro) | Parameterized robot-description macros |
| `message_filters` | [Humble API](https://docs.ros.org/en/humble/p/message_filters/) · [source](https://github.com/ros2/message_filters) | Optional synchronization primitives; do not confuse receipt-time matching with acquisition-time validity |
| Cyclone DDS | [Documentation](https://cyclonedds.io/docs/) · [source](https://github.com/eclipse-cyclonedds/cyclonedds) | Middleware configuration; selected by the default container |
| REP-103 | [Units and coordinate conventions](https://www.ros.org/reps/rep-0103.html) | SI units, axes and orientation conventions |
| REP-105 | [Mobile-platform coordinate frames](https://www.ros.org/reps/rep-0105.html) | `map`, `odom` and body-frame responsibilities |
| ROS Index | [Package directory](https://index.ros.org/) | Find package release/distribution information |

Local links: [message packages](../../ros2_ws/src/README.md),
[robot description](../../ros2_ws/src/unitree_g1_description/README.md),
[source contracts](../ONBOARD_SOCIAL_NAVIGATION.md).

## Navigation and mapping

The current Nav2 website uses **versioned Rolling documentation**. The links
below are useful explanations, but newer configuration keys and plugins may not
exist in Humble. Use the [Nav2 Humble source branch](https://github.com/ros-navigation/navigation2/tree/humble)
and local configuration for deployment details.

| Resource | Documentation / source | Role and purpose |
| --- | --- | --- |
| Nav2 | [Docs entry point](https://docs.nav2.org/) · [source](https://github.com/ros-navigation/navigation2) | Used: navigation framework and server/plugin architecture |
| Nav2 getting started | [Current walkthrough](https://docs.nav2.org/rolling/getting_started/) | Learning: installation and navigation concepts; Rolling examples |
| Nav2 tutorials | [Tutorial collection](https://docs.nav2.org/rolling/tutorials/) | Learning: custom plugins, SLAM and navigation behaviors |
| Robot setup | [First-time setup](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/) | Learning: TF, URDF, sensors, odometry and footprint |
| Configuration and tuning | [Parameter guide](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/) · [tuning guide](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/) | Learning: inspect tradeoffs and then check installed-version parameters |
| Simple Commander | [API guide](https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/) | Optional client pattern for requesting navigation tasks |
| SLAM Toolbox | [Repository and configuration guide](https://github.com/SteveMacenski/slam_toolbox) | Used: online mapping/localization |
| BehaviorTree.CPP | [Tutorials/docs](https://www.behaviortree.dev/) · [source](https://github.com/BehaviorTree/BehaviorTree.CPP) | Used through Nav2: select the BT library version bundled with Humble rather than assuming current v4 examples apply |
| `robot_localization` | [Repository](https://github.com/cra-ros-pkg/robot_localization) | Optional: EKF/UKF sensor-fusion reference, not Geoff-Sim's default odometry |
| FAST-LIO | [Repository](https://github.com/hku-mars/FAST_LIO) | Research candidate: LiDAR-inertial odometry; sensor/timing compatibility requires evaluation |
| ORB-SLAM3 | [Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3) | Optional research: visual/visual-inertial SLAM, not the current navigation estimator |
| RTAB-Map ROS | [Repository](https://github.com/introlab/rtabmap_ros) | Optional research: RGB-D/stereo mapping integration |

Local links: [perception/navigation collection](NAVIGATION.md),
[route planning](../DYNAMIC_ROUTE_PLANNING.md),
[social costmap](../../ros2_ws/src/unitree_social_costmap/README.md).

## Manipulation and control

| Resource | Documentation / source | Role and purpose |
| --- | --- | --- |
| MoveIt 2 | [Humble docs](https://moveit.picknik.ai/humble/index.html) · [source](https://github.com/moveit/moveit2) | Used by the earlier action-based arm-planning path |
| MoveIt tutorials | [Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html) · [tutorial source](https://github.com/moveit/moveit2_tutorials) | Learning: RViz, C++ planning and manipulation examples |
| Planning-scene monitor | [Humble concepts](https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html) | Learning: robot state, world geometry and collision-scene updates |
| ros2_control | [Humble getting started](https://control.ros.org/humble/doc/getting_started/getting_started.html) · [source](https://github.com/ros-controls/ros2_control) | Installed / reference: controller and hardware-interface architecture |
| ros2_controllers | [Humble docs](https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html) · [source](https://github.com/ros-controls/ros2_controllers) | Installed / reference: controller contracts and trajectory interfaces |
| ros2_control demos | [Repository](https://github.com/ros-controls/ros2_control_demos) | Learning: hardware interfaces, resource ownership and controller composition |
| OMPL | [Documentation](https://ompl.kavrakilab.org/) · [source](https://github.com/ompl/ompl) | Used through MoveIt: sampling-based planning concepts |
| Pinocchio | [Documentation](https://stack-of-tasks.github.io/pinocchio/) · [source](https://github.com/stack-of-tasks/pinocchio) | Optional: rigid-body kinematics/dynamics and Jacobian reference |
| Robotics Toolbox for Python | [Repository](https://github.com/petercorke/robotics-toolbox-python) | Optional learning: kinematics, dynamics and robot models |
| Modern Robotics | [Book, videos and exercises](https://modernrobotics.northwestern.edu/nu-gm-book-resource/) · [code](https://github.com/NxRLab/ModernRobotics) | Learning: transformations, twists, Jacobians, dynamics and planning |
| Grove-G1 | [Repository](https://github.com/Adyansh04/grove-g1) | Optional external stack: separate Jazzy environment and different hand/controller assumptions |

The default Geoff-Sim bridge publishes Unitree commands directly; installing
ros2_control packages does not make them the default hardware writer. The
[MoveIt/action path](MANIPULATION.md#robot-arms-and-action-interfaces) and newer
[sensor-driven transport workflow](../PAYLOAD_POLICY_TRAINING.md) are distinct.
Read the [Grove review](../GROVE_G1_REVIEW.md) before transferring designs or gains.

## MuJoCo and Unitree

| Resource | Documentation / source | Role and purpose |
| --- | --- | --- |
| MuJoCo | [Overview](https://mujoco.readthedocs.io/en/stable/overview.html) · [source](https://github.com/google-deepmind/mujoco) | Used: physics engine and model/data conventions |
| MuJoCo modeling | [Modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html) · [XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) | Used / learning: MJCF geometry, joints, sensors, contacts and actuators |
| MuJoCo Python | [Bindings and tutorial](https://mujoco.readthedocs.io/en/stable/python.html) · [tutorial notebook](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb) | Training / learning: model loading, stepping, viewers and data access; Colab may require Google sign-in |
| Unitree MuJoCo | [Repository](https://github.com/unitreerobotics/unitree_mujoco) | Used through the project fork and reviewed patches |
| Unitree ROS 2 | [Repository](https://github.com/unitreerobotics/unitree_ros2) | Used: Unitree ROS interfaces and examples |
| Unitree SDK2 | [Repository](https://github.com/unitreerobotics/unitree_sdk2) | Used: low-level DDS SDK |
| Unitree RL Lab | [Repository](https://github.com/unitreerobotics/unitree_rl_lab) | Used: official-policy ecosystem; the local downloader pins the deployed artifact hash |

Local links: [world generators](../../scripts/README.md), [patch workflow](../../patches/README.md),
[policy downloader](../../scripts/download_unitree_rl_lab_policy.sh).

## Learning and policy deployment

| Resource | Documentation / source | Role and purpose |
| --- | --- | --- |
| MJLab | [Docs](https://mujocolab.github.io/mjlab/) · [source](https://github.com/mujocolab/mjlab) | Training: manager-based locomotion environments |
| MuJoCo Warp | [Repository](https://github.com/google-deepmind/mujoco_warp) | Training: GPU-accelerated MuJoCo backend |
| NVIDIA Warp | [Documentation](https://nvidia.github.io/warp/) · [source](https://github.com/NVIDIA/warp) | Training: GPU kernels and simulation infrastructure |
| RSL-RL | [Repository](https://github.com/leggedrobotics/rsl_rl) | Training: on-policy RL ecosystem used by MJLab wrappers |
| PyTorch | [Tutorials](https://docs.pytorch.org/tutorials/) · [API](https://docs.pytorch.org/docs/stable/index.html) · [source](https://github.com/pytorch/pytorch) | Training: networks, gradients and optimization |
| ONNX | [Documentation](https://onnx.ai/onnx/) · [source](https://github.com/onnx/onnx) | Training/export: model graphs, operators and interchange |
| ONNX Runtime | [Documentation](https://onnxruntime.ai/docs/) · [source](https://github.com/microsoft/onnxruntime) | Used: policy inference and execution-provider configuration |
| Gymnasium | [Documentation](https://gymnasium.farama.org/) · [source](https://github.com/Farama-Foundation/Gymnasium) | Learning / environment ecosystem: observations, actions, reset and step contracts |
| Stable-Baselines3 | [Documentation](https://stable-baselines3.readthedocs.io/en/master/) · [source](https://github.com/DLR-RM/stable-baselines3) | Optional learning: accessible RL baselines; not the payload trainer used here |
| Isaac Lab | [Repository](https://github.com/isaac-sim/IsaacLab) | Optional comparison: separate simulation/training framework, not the MuJoCo runtime |

Local links: [training/calibration](../TRAINING_AND_CALIBRATION.md),
[payload trainer](../../scripts/g1_payload_training.py),
[policy contract](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/unitree_rl_lab_policy.py).
Policies are not interchangeable solely because they target a G1: observation
history, action meaning, joint ordering and gains belong to each policy contract.

## Perception, geometry and numerical libraries

| Library | Documentation / source | Role and useful tasks |
| --- | --- | --- |
| NumPy | [2.2 documentation](https://numpy.org/doc/2.2/) · [source](https://github.com/numpy/numpy) | Used: array operations, geometry and analysis; direct runtime version is pinned |
| SciPy | [Documentation](https://docs.scipy.org/doc/scipy/) · [source](https://github.com/scipy/scipy) | Optional numerical reference: optimization, spatial queries and signal processing |
| OpenCV | [4.x documentation](https://docs.opencv.org/4.x/) · [source](https://github.com/opencv/opencv) | Optional perception: camera calibration, images, features and geometry |
| ROS vision_opencv | [Repository](https://github.com/ros-perception/vision_opencv) | Optional ROS image/OpenCV bridge; choose the matching ROS branch |
| PCL | [Documentation](https://pointclouds.org/documentation/) · [source](https://github.com/PointCloudLibrary/pcl) | Optional perception: point-cloud filtering, registration and segmentation |
| Open3D | [Documentation](https://www.open3d.org/docs/release/) · [source](https://github.com/isl-org/Open3D) | Optional offline geometry: point clouds, registration and visualization |
| Eigen | [3.4 documentation](https://libeigen.gitlab.io/eigen/docs-3.4/) · [source](https://gitlab.com/libeigen/eigen) | C++ numerical reference: linear algebra used widely across robotics dependencies; match the installed major version |
| CasADi | [Documentation](https://web.casadi.org/docs/) · [source](https://github.com/casadi/casadi) | Optional optimization: automatic differentiation and optimal-control experiments |
| OSQP | [Documentation](https://osqp.org/docs/) · [source](https://github.com/osqp/osqp) | Optional optimization: quadratic-programming experiments |
| Matplotlib | [Documentation](https://matplotlib.org/stable/) | Optional analysis: plots and publication figures |
| pandas | [Documentation](https://pandas.pydata.org/docs/) | Optional analysis: tabular exploration of exported results |

The [project perception modules](NAVIGATION.md#follow-one-observation-through-the-source)
and [dependency files](../../docker/requirements.txt) identify what actually runs.
Listing a library here does not add it to the environment or change the onboard
input-source contract.

## Recording, debugging and profiling

| Tool | Documentation / source | Useful for |
| --- | --- | --- |
| rosbag2 | [Humble source](https://github.com/ros2/rosbag2/tree/humble) | Optional diagnostic captures and replay; preserve topic QoS and timestamps |
| RViz | [Source](https://github.com/ros2/rviz) | Used: robot, sensor, map and trajectory visualization |
| rqt | [Source](https://github.com/ros-visualization/rqt) | Optional ROS introspection and plugin tools |
| PlotJuggler | [Repository](https://github.com/PlotJuggler/PlotJuggler) | Optional time-series inspection and signal comparison |
| Foxglove | [Documentation](https://docs.foxglove.dev/docs) · [ROS bridge](https://github.com/foxglove/ros-foxglove-bridge) | Optional visualization/recording tooling; separate from the built-in results GUI |
| MCAP | [Format and tooling](https://mcap.dev/) | Optional recording-format reference for robotics data |
| ROS 2 tracing | [Repository](https://github.com/ros2/ros2_tracing) | Optional callback/runtime instrumentation; support depends on the installed ROS build |
| ros2doctor / ros2cli | [Humble source](https://github.com/ros2/ros2cli/tree/humble) | ROS command-line inspection and environment diagnostics |

For the project-specific evidence workflow, use the [results-review guide](../RESULTS_REVIEW.md),
[dashboard](../../tools/README.md), [run logger](../../ros2_ws/src/unitree_g1_mujoco_bridge/unitree_g1_mujoco_bridge/human_aware_run_logger.py)
and [calibration tools](../../scripts/README.md). Visual overlays and replay are
diagnostic aids; benchmark outcomes still come from the configured scorer.

## Build, test and community resources

| Resource | Documentation / source | Useful for |
| --- | --- | --- |
| Docker | [Getting started](https://docs.docker.com/get-started/) · [Compose](https://docs.docker.com/compose/) | Used: containers, source mounts and service lifecycle |
| colcon | [Documentation](https://colcon.readthedocs.io/en/released/) | Used: ROS workspace builds, testing and overlays |
| rosdep | [Documentation](https://docs.ros.org/en/independent/api/rosdep/html/) · [source](https://github.com/ros-infrastructure/rosdep) | Used: resolve package dependencies |
| vcstool | [Repository](https://github.com/dirk-thomas/vcstool) | Used in the source-tooling environment: import/export multi-repository workspaces |
| uv | [Documentation](https://docs.astral.sh/uv/) | Training/tooling: Python environment and dependency management |
| CMake / CTest | [Documentation](https://cmake.org/cmake/help/latest/) | Used: native builds and simulator checks |
| pytest | [Documentation](https://docs.pytest.org/en/stable/) | Used: Python regression tests |
| GoogleTest | [Documentation](https://google.github.io/googletest/) | Used by C++ package testing |
| GitHub Actions | [Documentation](https://docs.github.com/en/actions) | Used: CI workflows and build/test artifacts |
| Articulated Robotics | [Tutorials](https://articulatedrobotics.xyz/tutorials/) | Community learning: ROS robot setup, Docker and coordinate transforms; examples may use another distro |
| Robotics Stack Exchange | [Q&A](https://robotics.stackexchange.com/) | Community troubleshooting; check answer date and distribution |
| Open Robotics Discourse | [Forum](https://discourse.openrobotics.org/) | Community announcements, design discussion and ecosystem updates |

Local links: [development guide](../DEVELOPMENT.md), [verification workflow](../../.github/workflows/verification.yml),
[contributing](../../CONTRIBUTING.md), [issue tracker](https://github.com/DavidBatty95/Geoff-Sim/issues).

## Project pins and compatibility

| Local authority / declared source | What it identifies |
| --- | --- |
| [Submodule declarations](../../.gitmodules) | Checkout URLs; use parent gitlinks for the exact revisions |
| [Patch manifest](../../patches/manifest.json) | Reviewed source bases, patch checksums and resulting file hashes |
| [Dockerfile](../../docker/Dockerfile) / [Python pins](../../docker/requirements.txt) | Supported image dependencies and pinned direct Python packages |
| [Policy downloader](../../scripts/download_unitree_rl_lab_policy.sh) | Official policy URL and required checksum |
| [Geoff-Sim simulator fork](https://github.com/DavidBatty95/Geoff-Sim-unitree-mujoco) | Declared fork URL; unauthenticated check returned 404, so access/availability is unresolved |
| [Geoff-Sim MJLab fork](https://github.com/DavidBatty95/Geoff-Sim-mjlab) | Declared fork URL; unauthenticated check returned 404, so access/availability is unresolved |
| [Grove integration profile](../../integrations/grove_g1/README.md) | Optional checkout pin and runtime boundary; separate from the newer reviewed revision |

Use [the patch workflow](../../patches/README.md) and recorded experiment identity
to reproduce the maintained system. This directory does not recommend upgrading
every dependency to its newest release. Keep [license/attribution information](../../LICENSES.md)
with any adopted source or asset.

## Link maintenance

**Reachability review — 5 October 2026:** the directory's external URLs were
checked and moved Nav2, Eigen and PlotJuggler references updated. The two declared
project forks and project issue tracker returned 404 without authentication;
their access/availability remains unresolved. Colab loads a sign-in page, and
ROS documentation can return a browser challenge. These responses are labelled
instead of being treated as confirmed public content.

Prefer an official versioned guide and the canonical repository over a reposted
tutorial. Add a short purpose, project role and version note when extending the
directory. The [paper archive](RESEARCH.md) remains the single bibliography index;
this page covers software, practical tutorials and community resources.

The local documentation check validates repository targets and headings are
reviewed separately. External reachability is checked during documentation
updates; a successful HTTP response does not prove compatibility or validate a
tutorial's claims. Browser challenges and authenticated/private repositories
need separate interpretation. See the labelled fork entries above.

[Back to the library](README.md) · [Find project code](../REPOSITORY_GUIDE.md) · [Browse research papers](RESEARCH.md)
