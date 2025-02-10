# Moving Horizon Planning for Human-Robot Interaction
This repository provides an optimization-based robot trajectory planner (and control support for the UR10) that considers obstacles and humans in the environment.

We utilize a Moving Horizon Planning approach (similar to MPC) to optimize the robot trajectory based on cost functions regarding the environment.
The environment definition of this repository supports various types of obstacles:
   - static obstacles
   - dynamic obstacles
   - humans

Furthermore, the motion of dynamic obstacles and humans can be predicted and is considered in trajectory optimization. 
An uncertainty estimation approach is provided for an extrapolation approach to human motions to increase human safety.

The planner is provided as ROS packages for ROS Melodic and Noetic (Ubuntu 18 and Ubuntu 20).
Furthermore, a docker installation is provided for simple out-of-the-box usage of the planner (it may also run on systems other than Ubuntu).

Further information about the installation, usage, and other parts can be found in this repository in our [wiki](https://github.com/rst-tu-dortmund/mhp4hri/wiki).

## Feature Branch "Next-Best-Trajectory"
This branch presents a new feature for environment observation and has been uploaded to provide an extended version of the MHP4HRI.
The "Next-Best-Trajectory" feature allows the robot to observe the environment and plan a trajectory that optimizes the robot's view toward a point of interest (POI).
Therefore, we build up an online local information distribution in a voxel map (UfoMap) and use this information to plan a trajectory that optimizes the robot's view toward a POI.
Furthermore, we also provide a global ergodic reference option that uses an ergodic planner to deliver a trajectory that optimizes the robot's view over the whole environment as a reference for the local planner.

The feature is kept as a feature branch to keep the main branches clean and separate the main and new features.
Furthermore, a few limitations in comparison to the main branches are present:
- only ROS Noetic support
- no docker support (at least not tested)
- no full simulation support (the robot simulation is running, but we currently do not have a camera simulation to update the environment during a simulation)

Furthermore, the new feature extends the UfoMap implementation (https://github.com/UnknownFreeOccupied/ufomap, BSD 2-Clause License). Until the feature is merged into the main branch, the UfoMap is not in the references wiki page but only referenced here:

D. Duberg and P. Jensfelt, "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown," in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6411-6418, Oct. 2020, doi: 10.1109/LRA.2020.3013861. keywords: {Octrees;Three-dimensional displays;Solid modeling;Path planning;Collision avoidance;Robot sensing systems;Mapping;RGB-D perception;motion and path planning}, 

For the global ergodic reference, we use the ergodic planner from the following paper/GitHub (https://github.com/ialab-yale/time_optimal_ergodic_search) and the pomegranate library (https://pomegranate.readthedocs.io/en/latest/) for estimating a General mixture model as target distribution. The references are:

D. Dong, H. Berger, and I. Abraham, "Time Optimal Ergodic Search," in Robotics: Science and Systems XIX, Robotics: Science and Systems Foundation, Jul. 2023. doi: 10.15607/RSS.2023.XIX.082.

Schreiber, J. (2018). Pomegranate: fast and flexible probabilistic modeling in python. Journal of Machine Learning Research, 18(164), 1-6.

If you want to apply the feature with a real camera, you must use a camera that provides at least a point cloud and a color image. The feature is currently tested with an Azure Kinect camera. 
Microsoft delivers the corresponding ROS package under the MIT License (https://github.com/microsoft/Azure_Kinect_ROS_Driver).
It is not included by default so that every user can decide whether to get the package, depending on his camera or if already self-written drivers exist.

A new dependency in addition to the already listed is Eigen Rand: https://bab2min.github.io/eigenrand/v0.5.0/en/index.html, MIT License.

To use the GPU-accelerated version of the information distribution, you need to install the CUDA Toolkit (https://developer.nvidia.com/cuda-downloads). It is tested with version 11.8.

## Usage

For testing the feature, you can use our additional provided data which you can download from the following link: https://tu-dortmund.sciebo.de/s/0lG0yOYX3jG1dzs/download.
The data is a compressed ROS bag file.
Please decompress the file and place the decompressed bag in the new folder `mhp_ufomap/ufomap_bundled/config/`.

The new launch file `mhp_robot_ur10_example/ur_launch/ur10_sim_ufomap.launch` is already set up to use the bag file for testing:
```bash
roslaunch ur_launch ur10_sim_ufomap.launch
```


To start the global ergodic reference, you need to start the ergodic planner with the following command:
```bash
roslaunch ur_launch global_ergodic_reference.launch
```
To provide a global reference option that can also be used with other planners, we have already provided a version that includes OMPL (https://ompl.kavrakilab.org/) data formats. Therefore, this extension is required.

Afterward, you can start the robot task as usual with the following command:
```bash
rosservice call /start_all true
```

## New dependencies and references
The new feature requires the following additional dependencies:
- Eigen Rand: https://bab2min.github.io/eigenrand/v0.5.0/en/index.html, MIT License
- CUDA Toolkit: https://developer.nvidia.com/cuda-downloads, NVIDIA License
- UfoMap: https://github.com/UnknownFreeOccupied/ufomap, BSD 2-Clause License

   D. Duberg and P. Jensfelt, "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown," in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6411-6418, Oct. 2020, doi: 10.1109/LRA.2020.3013861. keywords: {Octrees;Three-dimensional displays;Solid modeling;Path planning;Collision avoidance;Robot sensing systems;Mapping;RGB-D perception;motion and path planning}

- Time Optimal Ergodic Search: https://github.com/ialab-yale/time_optimal_ergodic_search, unknown License, requires to download the repository on your own

   D. Dong, H. Berger, and I. Abraham, "Time Optimal Ergodic Search," in Robotics: Science and Systems XIX, Robotics: Science and Systems Foundation, Jul. 2023. doi: 10.15607/RSS.2023.XIX.082.

- pomegranate: https://pomegranate.readthedocs.io/en/latest/ (for estimating a General mixture model as target distribution), MIT License

   Schreiber, J. (2018). Pomegranate: fast and flexible probabilistic modeling in python. Journal of Machine Learning Research, 18(164), 1-6.

- Open Motion Planning Library (OMPL): https://ompl.kavrakilab.org/, BSD 3-Clause License

- Sub-sampling for the ergodic planner: M. Krämer and T. Bertram, "Improving Local Trajectory Optimization by Enhanced Initialization and Global Guidance," IEEE Access, vol. 10, pp. 29633–29645, 2022, doi: 10.1109/ACCESS.2022.3159233.

## Citation
If you use this repository for your research, please cite our papers:

H. Renz, M. Krämer, F. Hoffmann and T. Bertram, "Next-Best-Trajectory Planning of Robot Manipulators for Effective Observation and Exploration," 2025 IEEE International Conference on Robotics and Automation (ICRA), Atlanta, USA, 2025 [Accepted for publication]

Heiko Renz, Maximilian Krämer, and Torsten Bertram. 2024. Moving Horizon Planning for Human-Robot Interaction. In Proceedings of the 2024 ACM/IEEE International Conference on Human-Robot Interaction (HRI '24). Association for Computing Machinery, New York, NY, USA, 939–943. https://doi.org/10.1145/3610977.3637476

  

## Acknowledgments
The authors gratefully acknowledge the financial support of the German Research Foundation (DFG, Project number: 497071854)

