# Time-Optimal Ergodic Search



###    Please note that this repository has to be downloaded independently from the MHP4HRI repository due to missing license information.



Please clone/download the repository from the following link: <a href="https://github.com/ialab-yale/time_optimal_ergodic_search/tree/main">Time-Optimal Ergodic Search</a>

For the usage as global ergodic reference trajectory in the MHP4HRI project, please follow these steps:
1. Clone/download the repository from the link above.
2. Reuse the predefined folder structure and add all elements of the folder *time_opt_erg_lib* into this structure.
3. Note that the files *time_opt_erg_lib/obstacle.py* and *time_opt_erg_lib/target_distribution.py* have to be modified to be used as global ergodic reference trajectory in the MHP4HRI project. These classes rely on the base implementation but are slighlty adapted to this task.

    a. In *time_opt_erg_lib/obstacle.py* add the class:
    <details> <summary>ObstacleMHP</summary>
    
    ```python
    class ObstacleMHP(object):
        def __init__(self, obs_dict, buff=0.1, p=4):
            self._obs_dict = obs_dict
            self.half_dims = np.array(
                [
                    obs_dict["length_x"] / 2,
                    obs_dict["length_y"] / 2,
                    obs_dict["translation"][2] / 2,
                ]
            )
            self.pos = np.array(
                [
                    obs_dict["translation"][0] + self.half_dims[0],
                    obs_dict["translation"][1] + self.half_dims[1],
                    obs_dict["translation"][2] - self.half_dims[2],
                ]
            )
            self.dims = 2 * self.half_dims
            self.buff = buff
            self.th = obs_dict["rotation"]
            self.rot = np.identity(3)
            self.rotT = self.rot.T
            self.inv_rot = lambda p: self.rotT @ (self.pos - p)
            self.p = p

        def __getitem__(self, key):
            return self._obs_dict[key]

        def draw(self):
            rect = plt.Rectangle(self.pos - self.half_dims, self.dims[0], self.dims[1])
            return rect

        def draw3d(self):
            def get_cube():
                phi = np.arange(1, 10, 2) * np.pi / 4
                Phi, Theta = np.meshgrid(phi, phi)

                x = np.cos(Phi) * np.sin(Theta)
                y = np.sin(Phi) * np.sin(Theta)
                z = np.cos(Theta) / np.sqrt(2)
                return x, y, z

            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

            a = self.dims[0]
            b = self.dims[1]
            c = self.dims[2]
            x, y, z = get_cube()
            ax.plot_surface(
                x * a + self.pos[0] + self.half_dims[0],
                y * b + self.pos[1] + self.half_dims[0],
                z * c + self.pos[2] - self.half_dims[0],
            )
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.set_zlim(-2, 2)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")

            plt.show()

        def distance3(self, x):
            return 1.0 - np.linalg.norm(
                (x - self.pos) / (self.half_dims + self.buff), ord=2
            )

        def distance(self, x):
            return 1.0 - np.linalg.norm(
                (self.rotT @ (x - self.pos)) / (self.half_dims + self.buff), ord=4
            )                                               
    ```                                                                             
    </details>
    
    b. In *time_opt_erg_lib/target_distribution.py* add the class:
    <details> <summary>TargetDistributionMHP</summary>

    ```python
    class TargetDistributionMHP(object):
        def __init__(self, workspace_bnds=[[-1, 10], [-1, 10], [-1, 10]]) -> None:
            self._use_distribution = False
            self.n = 3

            axes = []
            for i in range(self.n):
                axes.append(np.linspace(workspace_bnds[i][0], workspace_bnds[i][1], 50))

            self.domain = np.meshgrid(axes[0], axes[1], axes[2])
            self._s = np.stack([X.ravel() for X in self.domain]).T
            self.evals = (self.p(self._s), self._s)

        def plot(self):
            fig = plt.figure()
            ax = fig.add_subplot(projection="3d")
            cmap = self.evals[0]

            # reduce cmap to values only above threshold
            plot_threshold = 0.3
            cmap = cmap[cmap > plot_threshold]

            # get belonging points to the cmap values from self.domain
            points = self.evals[1]
            points = points[self.evals[0] > plot_threshold]
            sctt = ax.scatter3D(
                points[:, 0],
                points[:, 1],
                points[:, 2],
                c=cmap,
            )
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")

            fig.colorbar(sctt, ax=ax, shrink=0.5, aspect=5)
            # plt.show()

        def set_p(self, distribution):
            self._use_distribution = True
            self.distribution = distribution
            self.evals = (np.array(self.p(self._s)), self._s)  # Update evals values

            # set all evals values to 0 if below 0.5

            # normalize evals values
            self.evals = (self.evals[0] / np.max(self.evals[0]), self.evals[1])
            # self.evals[0][self.evals[0] < 0.95] = 0

        def p(self, x):
            if self._use_distribution:
                return self.distribution.probability(x)

            else:
                return np.ones(x.shape[0])

        def update(self):
            pass
    ```
    </details>

    c. In *time_opt_erg_lib/dynamics.py* add the following class:
    <details> <summary>SingleIntegratorMHP</summary>

    ```python
    class SingleIntegratorMHP(object):
        def __init__(self) -> None:
            self.dt = 0.1
            self.n = 3
            self.m = 3
            B = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

            def dfdt(x, u):
                return B @ u

            def f(x, u):
                return x + self.dt * B @ u

            self.f = f
            self.dfdt = dfdt
    ```
    </details>

