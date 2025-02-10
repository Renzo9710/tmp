# Partially based and adapted on the code from: https://github.com/ialab-yale/time_optimal_ergodic_search/tree/main
# Modified/Extended to work with MHP4HRI by Heiko Renz, 2024
#!/usr/bin/env python3
import rospy
import time

import sys

# ROS nodes start in ~/.ros directory maybe need to change to your workspace
sys.path.append("../mhp/src/mhp/time_optimal_ergodic_search")

import numpy as np
import pyvista as pv
import pickle
import matplotlib.pyplot as plt
import seaborn
import rospkg

seaborn.set_style("whitegrid")

np.float = np.float64  # temp fix for following import
import ros_numpy
import yaml

from pomegranate.gmm import GeneralMixtureModel
from pomegranate.distributions import *

from mhp_robot.msg import MsgInfoPCLS
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point
from time_opt_erg_lib import (
    fourier_utils,
    target_distribution,
    dynamics,
    obstacle,
    cbf,
    cbf_utils,
    ergodic_metric,
    opt_solver,
)

from jax import vmap
import jax.numpy as jnp
import jax


class TimeOptErgodicPlan:

    def __init__(self, num_components=5, plot_pyvista=True):

        # Setup the GMM for the information distribution
        self._plot_pyvista = plot_pyvista
        self._num_components = num_components
        
        if self._plot_pyvista:
            self._first_plot = True
            self._plotter = pv.Plotter()
        else:
            self._first_plot = False
        # Only set by hand, if you want to plot the scatter of the points in a 3D matplotlib plot
        self._plot_scatter = False

        # Use all PCLs or only recent one
        self._use_all_pcls = True

        # Save the model
        self._save_model = False

        # Timing
        start = time.time()

        # Setup the basis functions, robot model and ergodic metric (remains same all the time)
        self.basis = fourier_utils.BasisFunc(n_basis=[10, 10, 10])
        self.robot_model = dynamics.SingleIntegratorMHP()
        self.erg_metric = ergodic_metric.ErgodicMetric(self.basis)

        # Load the obstacles
        utilitiesPath = rospkg.RosPack().get_path("mhp_robot")
        with open(
            utilitiesPath + "/config/obstacles/static_obstacles.yaml",
            "r",
        ) as file:
            obs_info = yaml.safe_load(file)

        # Init the CBF constraints
        self.obs = []
        self.cbf_constr = []
        obs_dict = {}
        obs_dict["length_x"] = 1.2
        obs_dict["length_y"] = 1.1
        obs_dict["translation"] = [0.65, -0.5, 1.6]
        obs_dict["rotation"] = [0.0, 0.0, 0.0]
        
        # uncomment to add an obstacle
        _ob = obstacle.ObstacleMHP(obs_dict, 0.3)  
        self.obs.append(_ob)
        self.cbf_constr.append(cbf_utils.sdf3cbf(self.robot_model.dfdt, _ob.distance3))
        
        # for obs_id in obs_info["static_obstacles"]:
        #     _ob = obstacle.Obstacle(obs_id, 0.2)  # add buffer to the obstacle
        #     # pos=np.array(obs_info[obs_name]['pos']),
        #     # half_dims=np.array(obs_info[obs_name]['half_dims']),
        #     # th=obs_info[obs_name]['rot']
        #     self.obs.append(_ob)
        #     self.cbf_constr.append(
        #         cbf_utils.sdf3cbf(self.robot_model.dfdt, _ob.distance3)
        #     )

        self.sphere_center = jnp.array([0.0, 0.0, 0.9273])
        self.reachability = 1.1

        # Print time to init the GMM
        print("Time to init GMM: ", time.time() - start)

        # Get parameters from server
        self.num_points = rospy.get_param("/ufomap_server_node/num_startpoints")

        # Subscribe to the PCL topic
        self._pcl_sub = rospy.Subscriber(
            "/ufomap_server_node/info_dist_cloud",
            MsgInfoPCLS,
            self._PCLCallback,
            queue_size=1,
        )
        self._erg_traj_pub = rospy.Publisher(
            "ergodic_trajectory", JointTrajectory, queue_size=10
        )
        self._new_start_sub = rospy.Subscriber(
            "/new_erg_start", Point, self._new_start_callback, queue_size=1
        )
        self._new_final_sub = rospy.Subscriber(
            "/new_erg_final", Point, self._new_final_callback, queue_size=1
        )

        self.x0 = np.array([0.5, -0.75, 0.8])
        self.xf = np.array([0.5, 0.75, 0.8])

    def _new_final_callback(self, msg):
        self.xf = np.array([msg.x, msg.y, msg.z])

    def _new_start_callback(self, msg):
        self.x0 = np.array([msg.x, msg.y, msg.z])

    def _PCLCallback(self, msg):
        # try since the GMM sometimes fails with decomposition errors --> repeat next incoming PCL
        try:
            self._fitGMM(msg)
        except Exception as e:
            print(e)

    def _fitGMM(self, PCL):
        # Timing
        start = time.time()

        # Get the points from the PCL and the weights
        if self._use_all_pcls:
            X = np.zeros([self.num_points * len(PCL.pcls), 4])
            for i in range(len(PCL.pcls)):
                # Get the points and weights
                X[i * self.num_points : (i + 1) * self.num_points] = (
                    self._get_xyzi_points(
                        ros_numpy.point_cloud2.pointcloud2_to_array(PCL.pcls[i])
                    ).astype("float32")
                )

                # Adapt weights regarding their age
                X[i * self.num_points : (i + 1) * self.num_points, 3] = X[
                    i * self.num_points : (i + 1) * self.num_points, 3
                ] * (
                    1 / (len(PCL.pcls) - i)
                )  # Last element has the highest weight since it is the most recent
        else:  # only use the last PCL
            X = np.zeros([self.num_points, 4])
            X[:] = self._get_xyzi_points(
                ros_numpy.point_cloud2.pointcloud2_to_array(PCL.pcls[-1])
            ).astype("float32")

        weights = X[:, 3]
        X = X[:, :3]

        # Get the workspace bounds
        decimals = 1
        mins = np.true_divide(np.floor(np.min(X, axis=0) * 10**decimals), 10**decimals)
        maxs = np.true_divide(np.ceil(np.max(X, axis=0) * 10**decimals), 10**decimals)
        self.workspace_bnds = [
            [mins[0], maxs[0]],
            [mins[1], maxs[1]],
            [mins[2], maxs[2]],
        ]

        # Init normals for the GMM for each component
        normals = []
        for i in range(self._num_components):
            normal = Normal()
            normals.append(normal)

        # Create a Multivariate Gaussian Distribution (change tolerance to speed up)
        model = GeneralMixtureModel(
            normals,
            tol=0.005,
            verbose=False,
        )
        # Fit the model to the data
        model.fit(X=X, sample_weight=weights)
        self.model = model
        # Save the model if desired
        if self._save_model:
            with open("gmm_model.pkl", "wb") as f:
                pickle.dump(model, f)
        # Print time to fit the GMM
        print("Time to fit GMM: ", time.time() - start)
        start2 = time.time()
        # Plan the ergodic trajectory
        self.trajectory = self._plan_trajectory()

        # Plotting Scatter of points (without weights, check RViz for coloured PCL)
        if self._plot_scatter:
            plt.figure(figsize=(10, 10))
            ax = plt.subplot(111, projection="3d")
            ax.scatter(X[:, 0], X[:, 1], X[:, 2])
            plt.axis(False)
            # plt.show()

        # Plotting the probability of each point in the grid and the mixture model
        if self._plot_pyvista:
            # get the grid of points in the target distribution
            x_2_ = np.array(
                list(
                    zip(
                        self.target_distribution.domain[0].flatten(),
                        self.target_distribution.domain[1].flatten(),
                        self.target_distribution.domain[2].flatten(),
                    )
                )
            )
            # get the probability of each point in the grid for the target distribution
            p2 = model.probability(x_2_).reshape(
                len(self.target_distribution.domain[0]),
                len(self.target_distribution.domain[1]),
                len(self.target_distribution.domain[2]),
            )

            # Create a StructuredGrid from PyVista
            meshp2 = pv.StructuredGrid(x_2_)
            meshp2.dimensions = self.target_distribution.domain[0].shape
            meshp2["p2"] = p2.flatten()

            # Plot the grid with the probability of each point
            if self._first_plot:
                self._first_plot = False
                self._plotter.show_grid()
                self._plotter.show_axes()
                # Add raw points
                actor3 = self._plotter.add_mesh(
                    pv.PolyData(X),
                    color="red",
                    render_points_as_spheres=True,
                    point_size=3,
                )
                # Add probability of each point in the grid
                actor4 = self._plotter.add_mesh(meshp2.contour(), opacity=0.2)

                # Add the trajectory
                points = np.array(self.trajectory["x"])
                tmp = np.zeros([len(points) - 1, 3])
                tmp[:, 0] = 2
                tmp[:, 1] = np.arange(len(points) - 1)
                tmp[:, 2] = np.arange(1, len(points))
                lines = np.hstack(tmp).astype(int)

                box = pv.Box(
                    (
                        self.obs[0].pos[0] - self.obs[0].half_dims[0],
                        self.obs[0].pos[0] + self.obs[0].half_dims[0],
                        self.obs[0].pos[1] - self.obs[0].half_dims[1],
                        self.obs[0].pos[1] + self.obs[0].half_dims[1],
                        self.obs[0].pos[2] - self.obs[0].half_dims[2],
                        self.obs[0].pos[2] + self.obs[0].half_dims[2],
                    )
                )
                actor6 = self._plotter.add_mesh(box, opacity=0.2, color="green")

                line = pv.PolyData(points, lines=lines)
                actor5 = self._plotter.add_mesh(line, color="red", line_width=5)

                # Add title
                self._plotter.add_title("GMM + PCL")
                self._plotter.show(interactive_update=True)
                # self._plotter.show()
                self._plotter.isometric_view()
            else:
                # self._plotter = pv.Plotter()
                self._plotter.clear()
                self._plotter.show_grid()
                self._plotter.show_axes()

                # Add raw points
                actor3 = self._plotter.add_mesh(
                    pv.PolyData(X),
                    color="red",
                    render_points_as_spheres=True,
                    point_size=3,
                )
                # Add probability of each point in the grid
                actor4 = self._plotter.add_mesh(meshp2.contour(), opacity=0.2)

                # Add the trajectory
                points = np.array(self.trajectory["x"])
                tmp = np.zeros([len(points) - 1, 3])
                tmp[:, 0] = 2
                tmp[:, 1] = np.arange(len(points) - 1)
                tmp[:, 2] = np.arange(1, len(points))
                lines = np.hstack(tmp).astype(int)

                box = pv.Box(
                    (
                        self.obs[0].pos[0] - self.obs[0].half_dims[0],
                        self.obs[0].pos[0] + self.obs[0].half_dims[0],
                        self.obs[0].pos[1] - self.obs[0].half_dims[1],
                        self.obs[0].pos[1] + self.obs[0].half_dims[1],
                        self.obs[0].pos[2] - self.obs[0].half_dims[2],
                        self.obs[0].pos[2] + self.obs[0].half_dims[2],
                    )
                )
                actor6 = self._plotter.add_mesh(box, opacity=0.2, color="green")

                line = pv.PolyData(points, lines=lines)
                actor5 = self._plotter.add_mesh(line, color="red", line_width=5)

                # Add title
                self._plotter.add_title("GMM + PCL")
                self._plotter.update(force_redraw=True)
                self._plotter.show()
                self._plotter.isometric_view()

        # Print time to plan the trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.joint_names = ["x", "y", "z"]
        trajectory_msg.points = []
        for i in range(len(self.trajectory["x"])):
            point = JointTrajectoryPoint()
            point.positions = self.trajectory["x"][i]
            point.velocities = self.trajectory["u"][i]
            point.time_from_start = rospy.Duration.from_sec(
                self.trajectory["tf"] * i / len(self.trajectory["x"])
            )
            trajectory_msg.points.append(point)
        self._erg_traj_pub.publish(trajectory_msg)
        print(
            "Time to plan trajectory: ",
            time.time() - start2,
            "Total time: ",
            time.time() - start,
        )

    def _get_xyzi_points(self, cloud_array, remove_nans=True, dtype=float):
        # Function adapated from: https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/point_cloud2.py

        """Pulls out x, y, z and Intensity columns from the cloud recordarray, and returns
        a 4xN matrix.
        """
        # remove crap points
        if remove_nans:
            mask = (
                np.isfinite(cloud_array["x"])
                & np.isfinite(cloud_array["y"])
                & np.isfinite(cloud_array["z"])
                & np.isfinite(cloud_array["intensity"])
            )
            cloud_array = cloud_array[mask]
        # pull out x, y, and z values
        points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
        points[..., 0] = cloud_array["x"]
        points[..., 1] = cloud_array["y"]
        points[..., 2] = cloud_array["z"]
        points[..., 3] = cloud_array["intensity"]

        return points

    def _plan_trajectory(self):
        # Set the optimization arguments
        opt_args = {
            "N": 50,
            # "x0": np.array([0.4, -0.5, 0.5]),
            # "x0": np.array([0.5, -0.8, 0.2]), # start point invalid due to y
            "x0": self.x0,
            # "xf": np.array([1.9, 1.0, 0.9]),
            "xf": self.xf,
            "erg_ub": 0.05,
            "alpha": 0.2,  # Increase alpha --> more distance to obstacles
        }
        print("x0: ", opt_args["x0"])
        # Set the target distribution
        self.target_distribution = target_distribution.TargetDistributionMHP(
            self.workspace_bnds
        )
        self.target_distribution.set_p(self.model)

        # Set the initial solution
        x = jnp.linspace(opt_args["x0"], opt_args["xf"], opt_args["N"], endpoint=True)
        u = jnp.zeros((opt_args["N"], self.robot_model.m))
        init_sol = {"x": x, "u": u, "tf": jnp.array(10.0)}

        # Update the optimization arguments with phik (Fourier coefficients)
        opt_args.update(
            {
                "phik": fourier_utils.get_phik(
                    self.target_distribution.evals, self.basis
                ),
            }
        )

        # Define the mapping function from states to workspace
        @vmap
        def emap(x):
            """Function that maps states to workspace"""
            return jnp.array(
                [
                    (x[0] - self.workspace_bnds[0][0])
                    / (self.workspace_bnds[0][1] - self.workspace_bnds[0][0]),
                    (x[1] - self.workspace_bnds[1][0])
                    / (self.workspace_bnds[1][1] - self.workspace_bnds[1][0]),
                    (x[2] - self.workspace_bnds[2][0])
                    / (self.workspace_bnds[2][1] - self.workspace_bnds[2][0]),
                ]
            )

        @vmap
        def emapSphere(x):
            """Function that maps states to workspace"""
            # jax.debug.print("x: {}", x)
            # jax.debug.print("x-sphere: {}", jnp.linalg.norm(x - sphere_center) / 1.3)
            return jnp.array(
                [jnp.linalg.norm(x - self.sphere_center) / self.reachability]
            )

        # Define the barier function
        def barrier_cost(e):
            """Barrier function to avoid robot going out of workspace"""
            return (jnp.maximum(0, e - 1) + jnp.maximum(0, -e)) ** 2

        # Define the loss function
        # @jit
        def loss(params, opt_args):
            x = params["x"]
            u = params["u"]
            tf = params["tf"]
            N = opt_args["N"]
            dt = tf / N
            e = emapSphere(x)  # adapt to sphere
            # jax.debug.print("e: {}", jnp.sum(barrier_cost(e)))
            """ Traj opt loss function, not the same as erg metric """
            return 10000000 * jnp.sum(barrier_cost(e)) + tf

        # Define the dynamic equality constraints
        def eq_constr(params, opt_args):
            """dynamic equality constriants"""
            x = params["x"]
            u = params["u"]

            x0 = opt_args["x0"]
            xf = opt_args["xf"]
            tf = params["tf"]
            N = opt_args["N"]
            dt = tf / N
            return jnp.vstack(
                [
                    x[0] - x0,
                    x[1:, :]
                    - (
                        x[:-1, :]
                        + dt * vmap(self.robot_model.dfdt)(x[:-1, :], u[:-1, :])
                    ),
                    x[-1] - xf,
                ]
            )

        # Define the inequality constraints
        def ineq_constr(params, opt_args):
            """inequality constraints"""
            x = params["x"]
            u = params["u"]
            phik = opt_args["phik"]
            tf = params["tf"]
            N = opt_args["N"]
            dt = tf / N
            e = emap(x)
            _cbf_ineq = [
                vmap(_cbf_ineq, in_axes=(0, 0, None, None))(
                    x, u, opt_args["alpha"], dt
                ).flatten()
                for _cbf_ineq in self.cbf_constr
            ]
            ck = fourier_utils.get_ck(e, self.basis, tf, dt)
            _erg_ineq = [
                jnp.array([self.erg_metric(ck, phik) - opt_args["erg_ub"], -tf])
            ]
            _ctrl_box = [(jnp.abs(u) - 0.5).flatten()]
            # jax.debug.print("_ctrl_box: {}", _ctrl_box)
            return jnp.concatenate(_erg_ineq + _ctrl_box + _cbf_ineq)
        
        solver = opt_solver.AugmentedLagrangeSolver(
            init_sol, loss, eq_constr, ineq_constr, opt_args, step_size=1e-3, c=1.0
        )
        solver.solve(max_iter=5000, eps=1e-5)
        sol = solver.get_solution()
        return sol


def distribution_node():
    rospy.init_node("distribution_node")

    TimeOptErgodicPlan(3, False)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    # Start the node
    distribution_node()
