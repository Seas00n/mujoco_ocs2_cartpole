# Cartpole MPC

[TOC]

## Modeling of Cartpole

| Symbol | Parameter                                            | Unit  |
| :----- | ---------------------------------------------------- | ----- |
| $\phi$ | Angle of Pole                                        | rad   |
| x      | Displacement of Cart                                 | m     |
| m, M   | Mass of Pole and Cart                                | Kg    |
| l      | Half Length of Pole                                  | m     |
| $I$    | Moment of inertial of Pole                           | Kgm^2 |
| P N    | Perpendicular and Horizontal Force From Cart to Pole | N     |


$$
M\ddot{x} + N = F\\
I\ddot{\phi} = -Plsin\phi-Nlcos\phi\\
N = m\frac{d^2}{dt^2}(x+lsin\phi)=m\ddot x+mlc_\phi\ddot\phi-mlc_\phi\dot\phi^2 \\
P = -mg - m\frac{d^2}{dt^2}(lcos\phi)=-mg+mls_\phi\ddot\phi+
mls_\phi\dot\phi^2
$$

Change into Standard Dynamic Equation and System Flow Map
$$
\begin{bmatrix}
I+ml^2&mlc_\phi\\
mlc_\phi&M+m
\end{bmatrix}\begin{bmatrix}
\ddot\phi\\
\ddot x
\end{bmatrix} = \begin{bmatrix}
ml^2c_{2\phi}\dot\phi+mgls_\phi\\
F+ml^2s_\phi\dot\phi^2
\end{bmatrix}
$$


$$
\mathbf{\dot x} = \begin{bmatrix}
\ddot\phi\\
\ddot x
\end{bmatrix} = \begin{bmatrix}
I+ml^2&mlc_\phi\\
mlc_\phi&M+m
\end{bmatrix}^{-1}\begin{bmatrix}
ml^2c_{2\phi}\dot\phi+mgls_\phi\\
F+ml^2s_\phi\dot\phi^2
\end{bmatrix}
$$



## Cost Function

$$
\begin{split}\label{eq:OCProblem}
\begin{cases}
\underset{\mathbf u(.)}{\min} \ \ \sum_i \phi_i(\mathbf x(t_{i+1})) + \displaystyle \int_{t_i}^{t_{i+1}} l_i(\mathbf x(t), \mathbf u(t), t) \, dt \\
    \text{s.t.} \ \ \mathbf x(t_0) = \mathbf x_0 \,\hspace{12em} \text{initial state} \\
    \ \ \ \ \ \dot{\mathbf x}(t) = \mathbf f(\mathbf x(t), \mathbf u(t), t) \hspace{8em} \text{system flow map} \\
    \ \ \ \ \ \ \mathbf{u_min}\leq\mathbf{u(t)}\leq\mathbf{u_max} \,\hspace{10em} \text{input constraint}\\
\end{cases}
\end{split} 
$$

$$
l_i = 0.5(x-x_{n})' Q (x-x_{n}) + 0.5(u-u_{n})' R (u-u_{n}) \\
\phi_i = 0.5(x-x_{n})' Q (x-x_{n})
$$



Construct Inequality Constraint using [Augmented Lagrangian Methods](https://www.him.uni-bonn.de/fileadmin/him/Section6_HIM_v1.pdf)

## Pinocchio Model Settings

`Pinocchio` Interface was created first using `urdf` file.

`info` record the number of states, mass of links and so on, which is used to generate `dynamicConstraints` for optimization problem.

```C++
 // create pinocchio interface
 pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile)));
 info = cp_interface::createCartPoleModelInfo(*pinocchioInterfacePtr_);
```

Finally, create `cartPoleSystemDynamics` which consists of `systemFlowMap`  

```
// Dynamics
problem_.dynamicsPtr.reset(new CartPoleSytemDynamics_pinocchio(info, libraryFolder, verbose));
```



## Mujoco MRT Interface

`MRT` Node is the interface between `MPC Node` and `Dummy Simulation Node`

`MRT_Dummy_ROS_Loop` gives the `ROS` communication process of simulation.

`Mujoco_MRT_Loop` inherit `MRT_Dummy_ROS_Loop` and override `modifyobseravation()`.

First you need to use `/path/to/mujoco/bin/compile` to compile the `cartpole.urdf` file which is just used in `pinocchio`.

```
/path/to/mujoco/bin/compile cartpole.urdf cartpole.xml
```

Some modification need to be applied to the `xml` file. (adding global options and actuator) Please check the `cartpole.xml` file.

The export environment variables such that `cmake ` can find `Mujoco`.

```
export MUJOCO_INSTALL_INCLUDE=/path/to/mujoco/include
export MUJOCO_INSTALL_BIN=/path/to/mujoco/build/bin
export MUJOCO_INSTALL_LIB=/path/to/mujoco/build/lib
```

 In `modificationObservation` function, `mjData.state` was set to the newest, and `mjData.ctrl` was updated to newest policy. Then use the `mjstep1` to update system, and update the observation.

There seems to be something wrong ... `mrt` system will apply `policy` once there is a message received and then step into `forward()` to update observation. If I use `mj_step1` to update the observation again in `modify()`method, policy will not align with the observation. This seems to cause some problem. Maybe I will write a new Interface Class to communicate with `mujoco` better.

The reason why I use `mujoco` as the back end is that the sensor interface of `mujoco` is simpler than that of `gazebo` . So if you just want to update sensor information like `camera` or `force sensor` you can do that in `modify()` method without `mj_step1`.

