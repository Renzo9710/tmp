# Manipulators for Occlusion- and Collision-Aware Predictive Human Tracking

We apply a Moving Horizon Planner from the literature (follow installation from the documentation and use the provided test data).

This repository adapts and extends the existing code.

The planner is provided as ROS packages for ROS Noetic (Ubuntu 20).

## Occlusion-Awareness
The "Occlusion-Awareness" feature allows the robot to track humans in it's workspace and to switch between tracking and observing the environment (one Point of Interest (POI)).
Therefore, we estimate occlusion areas around the target and use this information to plan a trajectory that optimizes the robot's view on the tracking target without being occluded. 
We consider not only the current robot/obstacles states but also future states to avoid occlusions of the target and to optimize the trajectory for a better view on the target.

The feature extends the UfoMap implementation (https://github.com/UnknownFreeOccupied/ufomap, BSD 2-Clause License). 

D. Duberg and P. Jensfelt, "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown," in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6411-6418, Oct. 2020, doi: 10.1109/LRA.2020.3013861.


If you want to apply the feature with a real camera, you must use a camera that provides at least a point cloud and a color image. The feature is currently tested with an Azure Kinect camera. 
Microsoft delivers the corresponding ROS package under the MIT License (https://github.com/microsoft/Azure_Kinect_ROS_Driver).
It is not included by default so that every user can decide whether to get the package, depending on his camera or if already self-written drivers exist.
Note that the Mediapipe Hands implementation only runs with a real camera and not with a simulation.

To use the GPU-accelerated version of the raycastings, you need to install the CUDA Toolkit (https://developer.nvidia.com/cuda-downloads). It is tested with version 11.8.


For human tracking we apply Mediapipe Hands (https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker?hl=de) and provide a wrapper to transform the results into our required messages. This is combined in the human-state-estimation-mediapipe package. Not that we used the official documentation as a starting point and adapted it. As model you need to download the hand_landmarker.task model on your own and copy it into the models folder.

As literature reference for Mediapipe Hands, please refer to:

F. Zhang, V. Bazarevsky, A. Vakunov, A. Tkachenka, G. Sung, C.-L. Chang, and M. Grundmann, “MediaPipe Hands: On-device Real-time Hand Tracking,” in IEEE/CVF Conf. on Computer Vision and Pattern Recognition Workshop Computer Vision for AR/VR, 2020.

## Usage

The launch file `mhp_robot_ur10_example/ur_launch/ur10_sim_ufomap.launch` is already set up to execute the simulation experiment 1:
```bash
roslaunch ur_launch ur10_sim_ufomap.launch
```

Afterward, you can start the robot task as usual with the following command:
```bash
rosservice call /start_all true
```
For activating/deactivating the occlusion prediction in Experiment 1, change line 111 in the `default_controller.yaml` file of the `mhp_planner` package to `use_occlusion_prediction: true` or `use_occlusion_prediction: false`.


For using mediapipe you need to add a camera driver on your own (We apply the Azure Kinect ROS Driver). 
Check the launch file `mhp_robot_ur10_example/ur_launch/ur10_sim_ufomap.launch` for information about topic names and the nodes that need to be started. 
The mediapipe wrapper is provided in the `human_state_estimation_mediapipe` package. 
For applying Mediapipe, you need to download the hand_landmarker.task model (https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task) on your own and copy it into the models folder of the `human_state_estimation_mediapipe` package. 

## Acknowledgments
Outcommented for review process.

