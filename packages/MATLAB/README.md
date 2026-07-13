# "mr" MATLAB Code Library Instructions #

## Installing the Library ##

Copy the "mr" folder to a known location on your computer. We will call the
path to this folder `$FOLDER_PATH/mr`.

## Importing the Library ##

To import the library, use `addpath` as

```
addpath('$FOLDER_PATH/mr')
```

This process is required for any program using this package.

## Using the Package ##

After importing the library, you should be able to use any function in the 
library. Taking the function `RotInv` for example, you can check the 
description and usage example of this function by running

```
help RotInv
```

As mentioned in the function usage example, you can try using this function
by running

```
R = [0, 0, 1; 1, 0, 0; 0, 1, 0];
invR = RotInv(R);
```

You should get the the variable `invR` whose value is the same as the output
shown in the function usage example.

To check the function list and which chapter in the book those functions 
belong to, use `help` as 

```
help mr
```

## Library Information ##

Author: Huan Weng, Bill Hunt, Mikhail Todes, Jarvis Schultz

Contact: huanweng@u.northwestern.edu

Package Version: 1.1.0

Matlab Version: R2017b

Tested in Matlab R2017b

# Modern Robotics: MATLAB Code Library (`mr`)

MATLAB library accompanying *Modern Robotics: Mechanics, Planning, and Control* by Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017, [modernrobotics.org](http://modernrobotics.org).

The functions below are organized according to the relevant chapter in the book, following the structure of the library documentation in [`doc/MRlib.pdf`](../../../doc/MRlib.pdf). Each `.m` file contains a comment header with an example use.

## Notation

| Math symbol | Variable | Description |
|---|---|---|
| R | `R` | 3×3 rotation matrix in SO(3) |
| ω | `omg` | 3-vector angular velocity |
| ω̂ | `omghat` | 3-vector unit rotation axis |
| θ | `theta` | Angle of rotation about an axis, or distance along a screw axis |
| ω̂θ | `expc3` | 3-vector of exponential coordinates for rotation |
| [ω], [ω̂θ] | `so3mat` | 3×3 skew-symmetric so(3) representation |
| p | `p` | 3-vector position in space |
| T | `T` | 4×4 transformation matrix in SE(3) corresponding to (R, p) |
| [Ad_T] | `AdT` | 6×6 adjoint representation of T ∈ SE(3) |
| V | `V` | 6-vector twist (ω, v) |
| S | `S` | Normalized 6-vector screw axis (ω, v) |
| Sθ | `expc6` | 6-vector of exponential coordinates for rigid-body motion |
| [V], [Sθ] | `se3mat` | 4×4 se(3) representation |
| M | `M` | End-effector home (zero position) configuration in SE(3) |
| B_i | `Blist` | Joint screw axes in the end-effector frame at the home position |
| S_i | `Slist` | Joint screw axes in the space frame at the home position |
| J_b, J_s | `Jb`, `Js` | 6×n body / space manipulator Jacobian |
| θ | `thetalist` | n-vector of joint variables (`thetamat`: N×n trajectory matrix) |
| θ̇ | `dthetalist` | n-vector of joint velocities (`dthetamat`: N×n matrix) |
| θ̈ | `ddthetalist` | n-vector of joint accelerations (`ddthetamat`: N×n matrix) |
| g | `g` | 3-vector gravitational acceleration |
| M_{i-1,i} | `Mlist` | Link frame {i} relative to {i-1} at the home position |
| F_tip | `Ftip` | 6-vector wrench applied by the end-effector (`Ftipmat`: N×6 matrix) |
| G_i | `Glist` | 6×6 spatial inertia matrices of the links |
| τ | `taulist` | n-vector of joint forces/torques (`taumat`: N×n matrix) |
| [ad_V] | `adV` | 6×6 adjoint representation of a twist, for Lie brackets |
| T_f | `Tf` | Total time of a motion, rest to rest |
| Δt | `dt` | Timestep |
| K_p, K_i, K_d | `Kp`, `Ki`, `Kd` | Scalar feedback gains |
| — | `intRes` | Number of integration steps per timestep Δt (positive integer) |

Variables with a `d` suffix (`thetalistd`, `dthetamatd`, …) are reference/desired values; a `tilde` infix (`gtilde`, `Mtildelist`, `Gtildelist`) marks possibly incorrect models used by a controller; `eint` is the time-integral of joint errors; `eomg` and `ev` are inverse-kinematics tolerances on orientation and position error; `thetalist0` is an initial IK guess.

## Basic helper functions

*Not documented in MRlib.pdf.*

### `judge = NearZero(near)`
Returns true (1) if the scalar `near` is small enough (magnitude < 1e-6) to be treated as zero.

### `norm_v = Normalize(V)`
Scales the vector `V` to a unit vector.

## Chapter 3: Rigid-Body Motions

### `invR = RotInv(R)`
Inverse of rotation matrix `R`. For efficiency, computed as the transpose rather than a matrix inverse.

### `so3mat = VecToso3(omg)`
Converts a 3-vector `omg` to the corresponding 3×3 skew-symmetric matrix in so(3).

### `omg = so3ToVec(so3mat)`
Converts a 3×3 skew-symmetric so(3) matrix back to the corresponding 3-vector.

### `[omghat, theta] = AxisAng3(expc3)`
Extracts from the exponential coordinates for rotation `expc3` = ω̂θ the unit rotation axis `omghat` and the rotation angle `theta`.

### `R = MatrixExp3(so3mat)`
Matrix exponential of `so3mat` = [ω̂θ]: the rotation matrix achieved by rotating about ω̂ by θ from the identity orientation.

### `so3mat = MatrixLog3(R)`
Matrix logarithm of rotation matrix `R`: the corresponding so(3) representation of exponential coordinates.

### `d = DistanceToSO3(mat)`
Measure of the distance of a 3×3 matrix `mat` from SO(3). If det(mat) > 0, computed as the Frobenius norm ‖matᵀmat − I‖_F; otherwise a large value is returned.

### `judge = TestIfSO3(mat)`
Returns 1 (true) if `mat` is a rotation matrix (element of SO(3)), 0 otherwise. Calls `DistanceToSO3` and tests the result against a small threshold.

### `R = ProjectToSO3(mat)`
Returns the rotation matrix closest to `mat`. Only appropriate for matrices "close" to SO(3), e.g. results that have drifted due to roundoff error.

### `T = RpToTrans(R, p)`
Assembles rotation matrix `R` and position `p` into a homogeneous transformation matrix T ∈ SE(3).

### `[R, p] = TransToRp(T)`
Extracts the rotation matrix `R` and position `p` from transformation matrix `T`.

### `invT = TransInv(T)`
Inverse of transformation matrix `T`. Uses the structure of transformation matrices to avoid a general matrix inverse, for efficiency.

### `se3mat = VecTose3(V)`
Converts a 6-vector `V` (e.g. a twist) to the corresponding 4×4 se(3) matrix.

### `V = se3ToVec(se3mat)`
Converts a 4×4 se(3) matrix back to the corresponding 6-vector.

### `AdT = Adjoint(T)`
The 6×6 adjoint representation [Ad_T] of transformation matrix `T`.

### `S = ScrewToAxis(q, s, h)`
Normalized screw axis S = (ω, v) from a point `q` on the axis, a unit direction vector `s`, and the pitch `h`.

### `[S, theta] = AxisAng6(expc6)`
Extracts from the exponential coordinates for rigid-body motion `expc6` = Sθ the normalized screw axis `S` and the distance `theta` traveled along/about it.

### `T = MatrixExp6(se3mat)`
Matrix exponential of `se3mat` = [Sθ]: the transformation matrix achieved by traveling along/about screw axis S a distance θ from the identity configuration.

### `se3mat = MatrixLog6(T)`
Matrix logarithm of transformation matrix `T`: the corresponding se(3) representation of exponential coordinates.

### `d = DistanceToSE3(mat)`
Measure of the distance of a 4×4 matrix `mat` from SE(3), based on the Frobenius norm of a matrix built from RᵀR (top-left 3×3 block) and the bottom row of `mat`. Returns a large value if det(R) is not positive.

### `judge = TestIfSE3(mat)`
Returns 1 if `mat` is a transformation matrix (element of SE(3)), 0 otherwise. Calls `DistanceToSE3` and tests the result against a small threshold.

### `T = ProjectToSE3(mat)`
Returns the transformation matrix closest to `mat`: the rotation block is projected to SO(3) with `ProjectToSO3`, the translation is kept, and the bottom row is set to [0 0 0 1]. Only appropriate for matrices "close" to SE(3).

## Chapter 4: Forward Kinematics

### `T = FKinBody(M, Blist, thetalist)`
Forward kinematics in the body frame: the end-effector configuration T ∈ SE(3) given the home configuration `M`, the joint screw axes `Blist` expressed in the end-effector frame, and joint values `thetalist`.

### `T = FKinSpace(M, Slist, thetalist)`
Forward kinematics in the space frame: the end-effector configuration T ∈ SE(3) given the home configuration `M`, the joint screw axes `Slist` expressed in the space frame, and joint values `thetalist`.

## Chapter 5: Velocity Kinematics and Statics

### `Jb = JacobianBody(Blist, thetalist)`
The body Jacobian J_b(θ) ∈ ℝ^(6×n) given the joint screw axes `Blist` in the end-effector frame and joint values `thetalist`.

### `Js = JacobianSpace(Slist, thetalist)`
The space Jacobian J_s(θ) ∈ ℝ^(6×n) given the joint screw axes `Slist` in the space frame and joint values `thetalist`.

## Chapter 6: Inverse Kinematics

### `[thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)`
Numerical inverse kinematics in the body frame using iterative Newton–Raphson root finding from the initial guess `thetalist0`. Returns joint variables `thetalist` achieving the desired end-effector configuration `T` within the orientation tolerance `eomg` and position tolerance `ev`, and a logical `success` flag. If the maximum number of iterations (hardcoded in the function) is reached first, the last iterate is returned with `success = false`.

### `[thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev)`
Equivalent to `IKinBody`, except the joint screw axes are specified in the space frame.

## Chapter 8: Dynamics of Open Chains

These functions calculate and simulate the dynamics of a serial-chain manipulator of the form

τ = M(θ)θ̈ + c(θ, θ̇) + g(θ) + Jᵀ(θ)F_tip

### `adV = ad(V)`
The 6×6 matrix [ad_V] of the twist `V`, used to calculate the Lie bracket [ad_V1]V2.

### `taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)`
The joint forces/torques `taulist` required to achieve the given joint positions, velocities, and accelerations, using forward-backward Newton–Euler iterations.

### `M = MassMatrix(thetalist, Mlist, Glist, Slist)`
The numerical inertia matrix M(θ) of an n-joint serial chain at configuration `thetalist`. Calls `InverseDynamics` n times, each with a unit acceleration at one joint and all other inputs zero, building the mass matrix column by column.

### `c = VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)`
The vector c(θ, θ̇) of Coriolis and centripetal terms. Calls `InverseDynamics` with g = 0, F_tip = 0, and θ̈ = 0.

### `grav = GravityForces(thetalist, g, Mlist, Glist, Slist)`
The joint forces/torques required to balance gravity at `thetalist`. Calls `InverseDynamics` with θ̇ = θ̈ = 0 and F_tip = 0.

### `JTFtip = EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist)`
The joint forces/torques Jᵀ(θ)F_tip required to create the end-effector wrench `Ftip`. Calls `InverseDynamics` with g = 0 and θ̇ = θ̈ = 0.

### `ddthetalist = ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)`
The resulting joint accelerations θ̈, computed by solving M(θ)θ̈ = τ − c(θ, θ̇) − g(θ) − Jᵀ(θ)F_tip.

### `[thetalistNext, dthetalistNext] = EulerStep(thetalist, dthetalist, ddthetalist, dt)`
Joint variables and velocities after one timestep `dt` of first-order Euler integration.

### `taumat = InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, Ftipmat, Mlist, Glist, Slist)`
The N×n matrix of joint forces/torques required to move the serial chain along the trajectory specified by the N×n matrices `thetamat`, `dthetamat`, `ddthetamat` and the N×6 end-effector wrench history `Ftipmat` (pass 0 for no tip forces). Uses `InverseDynamics` at each timestep.

### `[thetamat, dthetamat] = ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes)`
Simulates the motion of a serial chain given an open-loop history of joint forces/torques `taumat` (N×n, one row per timestep `dt`), starting from initial state (`thetalist`, `dthetalist`). Numerically integrates using `ForwardDynamics` with `intRes` Euler steps per timestep. Returns the resulting joint variable and velocity trajectories.

## Chapter 9: Trajectory Generation

### `s = CubicTimeScaling(Tf, t)`
The path parameter s(t) for a third-order polynomial motion that begins (s(0) = 0) and ends (s(Tf) = 1) at zero velocity.

### `s = QuinticTimeScaling(Tf, t)`
The path parameter s(t) for a fifth-order polynomial motion that begins and ends at zero velocity and zero acceleration.

### `traj = JointTrajectory(thetastart, thetaend, Tf, N, method)`
A straight-line trajectory in joint space as an N×n matrix, from `thetastart` to `thetaend` in time `Tf`, sampled at N points. `method` selects the time scaling: 3 for cubic, 5 for quintic.

### `traj = ScrewTrajectory(Xstart, Xend, Tf, N, method)`
A trajectory from end-effector configuration `Xstart` to `Xend` as a list of N SE(3) matrices, corresponding to a screw motion about a constant screw axis. `method` selects cubic (3) or quintic (5) time scaling.

### `traj = CartesianTrajectory(Xstart, Xend, Tf, N, method)`
Similar to `ScrewTrajectory`, except the origin of the end-effector frame follows a straight line, decoupled from the rotational motion.

## Chapter 11: Robot Control

These two functions use the computed torque controller

τ = M̂(θ)(θ̈_d + K_p θ_e + K_i ∫θ_e(t)dt + K_d θ̇_e) + ĥ(θ, θ̇)

to control the motion of a serial chain in free space, where ĥ(θ, θ̇) models the centripetal, Coriolis, and gravitational forces and M̂(θ) models the mass matrix.

### `taulist = ComputedTorque(thetalist, dthetalist, eint, g, Mlist, Glist, Slist, thetalistd, dthetalistd, ddthetalistd, Kp, Ki, Kd)`
The joint forces/torques computed by the computed torque controller at the current instant, given the current state (`thetalist`, `dthetalist`), the integrated joint error `eint`, the reference state (`thetalistd`, `dthetalistd`, `ddthetalistd`), and scalar PID gains `Kp`, `Ki`, `Kd` (identical for each joint).

### `[taumat, thetamat] = SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, Slist, thetamatd, dthetamatd, ddthetamatd, gtilde, Mtildelist, Gtildelist, Kp, Ki, Kd, dt, intRes)`
Simulates the performance of a computed torque control law operating on a serial chain, using `ComputedTorque`, `ForwardDynamics`, and numerical integration (`intRes` Euler steps per timestep `dt`). Disturbances come from initial position/velocity errors, the possibly incorrect models `gtilde`, `Mtildelist`, `Gtildelist`, and integration error. Returns the commanded joint forces/torques `taumat` and the actual joint trajectory `thetamat` (to compare against the desired `thetamatd`), and plots the actual and desired joint variables.

