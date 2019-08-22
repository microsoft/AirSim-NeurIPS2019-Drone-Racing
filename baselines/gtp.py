#!/usr/bin/env python
import numpy as np
import cvxpy as cp
from scipy.interpolate import CubicSpline, CubicHermiteSpline
import airsimneurips as airsim

gate_dimensions = [1.6, 1.6]

gate_facing_vector = airsim.Vector3r(x_val=0, y_val=1, z_val=0)


def rotate_vector(q, v):
    v_quat = v.to_Quaternionr()
    v_rotated_ = q * v_quat * q.inverse()
    return airsim.Vector3r(x_val=v_rotated_.x_val, y_val=v_rotated_.y_val, z_val=v_rotated_.z_val)


class SplinedTrack:
    """This class represents a Track defined by Gates.
    A spline is fitted through the Gates with tangential constraints.
    This spline is then sampled at 2048 points.
    """

    def __init__(self, gate_poses):
        self.gates = gate_poses

        self.n_gates = np.size(gate_poses, 0)
        positions = np.array([pose.position.to_numpy_array() for pose in gate_poses])
        dists = np.linalg.norm(positions[1:, :] - positions[:-1, :], axis=1)
        self.arc_length = np.zeros(shape=self.n_gates)
        self.arc_length[1:] = np.cumsum(dists)

        # Tangents from quaternion
        # by rotating default gate direction with quaternion
        self.tangents = np.zeros(shape=(self.n_gates, 3))
        for i, pose in enumerate(gate_poses):
            self.tangents[i, :] = rotate_vector(pose.orientation, gate_facing_vector).to_numpy_array()
        self.track_spline = CubicHermiteSpline(self.arc_length, positions, self.tangents, axis=0)

        # Gate width to track (half) width
        gate_widths = [gate_dimensions[0] / 2.0 for gate in gate_poses]
        gate_heights = [gate_dimensions[1] / 2.0 for gate in gate_poses]

        self.track_width_spline = CubicSpline(self.arc_length, gate_widths, axis=0)
        self.track_height_spline = CubicSpline(self.arc_length, gate_heights, axis=0)

        # Sample 2048 points, the 2048 are arbitrary and should really be a parameter
        taus = np.linspace(self.arc_length[0], self.arc_length[-1], 2048)

        self.track_centers = self.track_spline(taus)
        self.track_tangents = self.track_spline.derivative(nu=1)(taus)

        self.track_tangents /= np.linalg.norm(self.track_tangents, axis=1)[:, np.newaxis]

        self.track_normals = np.zeros_like(self.track_tangents)
        self.track_normals[:, 0] = -self.track_tangents[:, 1]
        self.track_normals[:, 1] = self.track_tangents[:, 0]

        self.track_normals /= np.linalg.norm(self.track_normals, axis=1)[:, np.newaxis]

        self.track_widths = self.track_width_spline(taus)
        self.track_heights = self.track_height_spline(taus)

    def track_frame_at(self, p):
        """Find closest track frame to a reference point p.
        :param p: Point of reference
        :return: Index of track frame, track center, tangent and normal.
        """
        i = np.linalg.norm(self.track_centers - p, axis=1).argmin()
        return i, self.track_centers[i], self.track_tangents[i], self.track_normals[i], \
               self.track_widths[i], self.track_heights[i]


class IBRController:
    """
    OVERVIEW:
    Given the state of both drones 0 and 1, this controller iteratively computes trajectories for both drones, that are
    collision-free and stay within the track and conform to the dynamical model (maximal speed).
    These trajectories are specified as 'n_steps' points every 'dt' seconds.

    ITERATIVE BEST RESPONSE:
    Initially, these trajectories are sampled to follow the track (for more information on the track see above).
    Then, fixing the trajectory for drone 1, the trajectory for drone 0 is optimized.
    Next, vice versa, the trajectory for drone 1 is optimized while keeping the trajectory for drone 0 fixed.
    This is done iteratively for a fixed number of iterations.
    The hope is that eventually we arrive at a fixed-point, i. e. after optimizing for both drones we get the same
    trajectories again.

    COMPUTING THE BEST RESPONSE:
    Since some of the constraints are non-convex quadratic constraint (namely the non-collision constraint),
    the optimized trajectory is found in an iteratively fashion (sequential quadratic program, short SQP) by linearizing
    the non-collision constraints around the current guess.
    In case no feasible solution is found, the problem is relaxed by turning constraints into objectives.

    PARAMETERS:
     i_ego         The index of the 'ego' drone
     i_opp         The index of the opponent drone
     dt            Sampling time for trajectories
     n_steps       Length of the trajectories
     n_game_iters  Number of game iterations (how often the best response is computed for drone i_0)
     n_sqp_iters   Number of SQP iterations (how often the constraints are linearized and the optimization is solved)
     drone_params
       r_coll      Collision radius, see competition guidelines.
       r_safe      Safety radius, see competition guidelines
       v_max       Maximal velocity, determines how far waypoints can be apart
       a_max       Maximal acceleration, not used here.
    """

    def __init__(self, params, drone_params, gate_poses):
        self.dt = params.dt
        self.n_steps = params.n
        self.drone_params = drone_params
        self.track = SplinedTrack(gate_poses)

        # These are some parameters that could be tuned.
        # They control how the safety penalty and the relaxed constraints are weighted.
        self.nc_weight = 2.0
        self.nc_relax_weight = 128.0
        self.track_relax_weight = 16.0

    def init_trajectory(self, i_0, p_0):
        """Initialize Trajectory along the track tangent
        Based on the start position p_0, return an initial guess for a trajectory
        This is simply a line following the tangent of the track with maximal speed

        :param i_0: The index of the drone to initialize the trajectory for.
        :param p_0: The current position of the drone
        :return: A trajectory of length self.n_steps
        """
        v_ego = self.drone_params[i_0]["v_max"]
        trajectory = np.zeros(shape=(self.n_steps, 3))
        p = np.copy(p_0)  # Copy state as it gets modified throughout the loop
        for k in range(self.n_steps):
            idx, c, t, n, width, height = self.track.track_frame_at(p)
            p += self.dt * v_ego * t
            trajectory[k, :] = p
        return trajectory

    def best_response(self, i_ego, state, trajectories):
        """Based on current trajectories, compute the best-response (BR) of player i_ego.

        This is done by solving an optimization problem over the trajectory
          p_ego[0], p_ego[1], ..., p_ego[N]
        for drone i_ego maximizing the progress along the track,
        subject to
          - dynamical constraints
            ||p_ego[k+1] - p_ego[k]|| <= v_max*dt
          - stay-within-track constraints
                  |n'(p[k] - c)| <= width,
             |[0 0 1](p[k] - c)| <= height,
            where n is the track normal (a vector on the xy-plane pointing perpendicular to the track tangent),
              and c is the track center
          - non-collision constraints
            ||p_ego[k] - p_opp[k]|| >= r_coll * 2

        The progress along the track is approximated by the progress along the tangent
        of the last point of the trajectory, i. e. t'p[k].
        The non-collision constraints are non convex and are hence linearized. Instead of requiring the ego drone to
        stay outside a circle, the drone is constrained to be in a half-plane tangential to that circle.
        In addition to optimizing track progress, the penalty of violating the safety radius is accounted for.

        :param i_ego: The drone index of the ego drone.
        :param state: The current state (positions) of the two drones.
        :param trajectories: The current trajectories
        :return: Optimized trajectory of player i_ego
        """
        i_opp = (i_ego + 1) % 2
        v_ego = self.drone_params[i_ego]["v_max"]
        r_coll_ego = self.drone_params[i_ego]["r_coll"]
        r_coll_opp = self.drone_params[i_opp]["r_coll"]
        r_safe_ego = self.drone_params[i_ego]["r_safe"]
        r_safe_opp = self.drone_params[i_opp]["r_safe"]
        d_coll = r_coll_ego + r_coll_opp
        d_safe = r_safe_ego + r_safe_opp

        p = cp.Variable(shape=(self.n_steps, 3))

        # === Dynamical Constraints ===
        # ||p_0 - p[0]|| <= v*dt
        init_dyn_constraint = cp.SOC(cp.Constant(v_ego * self.dt), cp.Constant(state[i_ego, :]) - p[0, :])

        # ||p[k+1] - p[k]|| <= v*dt
        dyn_constraints = [init_dyn_constraint] + [
            cp.SOC(cp.Constant(v_ego * self.dt), p[k + 1, :] - p[k, :]) for k in range(self.n_steps - 1)]

        # === Track Constraints ===
        track_constraints = []
        track_obj = cp.Constant(0)
        # exponentially decreasing weight
        track_objective_exp = 0.5

        for k in range(self.n_steps):
            idx, c, t, n, width, height = self.track.track_frame_at(trajectories[i_ego][k, :])
            track_constraints.append(n.T @ p[k, :] - n.dot(c) <= width - r_coll_ego)
            track_constraints.append(n.T @ p[k, :] - n.dot(c) >= -(width - r_coll_ego))
            # 3D Height. This assumes that gates are not rolled (i. e. aligned with the z-axis)
            track_constraints.append(p[k, 2] - c[2] <= height - r_coll_ego)
            track_constraints.append(p[k, 2] - c[2] >= -(height - r_coll_ego))

            track_obj += (track_objective_exp ** k) * (
                    cp.pos(n.T @ p[k, :] - n.dot(c) - (width - r_coll_ego)) +
                    cp.pos(-(n.T @ p[k, :] - n.dot(c) + (width - r_coll_ego))))

        # === Non-Collision Constraints ===
        nc_constraints = []
        nc_obj = cp.Constant(0)
        nc_relax_obj = cp.Constant(0)
        # exponentially decreasing weight
        non_collision_objective_exp = 0.5

        for k in range(self.n_steps):
            p_opp = trajectories[i_opp][k, :]
            p_ego = trajectories[i_ego][k, :]

            # Compute beta, the normal direction vector pointing from the ego's drone position to the opponent's
            beta = p_opp - p_ego
            if np.linalg.norm(beta) >= 1e-5:
                # Only normalize if norm is large enough
                beta /= np.linalg.norm(beta)

            #     n.T * (p_opp - p_ego) >= d_coll
            nc_constraints.append(beta.dot(p_opp) - beta.T @ p[k, :] >= d_coll)

            # For normal non-collision objective use safety distance
            nc_obj += (non_collision_objective_exp ** k) * cp.pos(d_safe - (beta.dot(p_opp) - beta.T @ p[k, :]))
            # For relaxed non-collision objective use collision distance
            nc_relax_obj += (non_collision_objective_exp ** k) * cp.pos(d_coll - (beta.dot(p_opp) - beta.T @ p[k, :]))

        # Take the tangent t at the last trajectory point
        # This serves as an approximation to the total track progress
        _, _, t, _, _, _ = self.track.track_frame_at(trajectories[i_ego][-1, :])
        obj = -t.T @ p[-1, :]

        # Create the problem in cxvpy and solve it.
        prob = cp.Problem(cp.Minimize(obj + self.nc_weight * nc_obj),
                          dyn_constraints + track_constraints + nc_constraints)
        prob.solve()

        if np.isinf(prob.value):
            print("Relaxing track constraints")
            # If the problem is not feasible, relax track constraints
            # Assert it is indeed an infeasible problem and not unbounded (in which case value is -inf).
            # (The dynamical constraints keep the problem bounded.)
            assert prob.value >= 0.0

            # Solve relaxed problem (track constraint -> track objective)
            relaxed_prob = cp.Problem(cp.Minimize(obj + self.nc_weight * nc_obj + self.track_relax_weight * track_obj),
                                      dyn_constraints + nc_constraints)
            relaxed_prob.solve()

            if np.isinf(relaxed_prob.value):
                print("Relaxing non collision constraints")
                # If the problem is still infeasible, relax non-collision constraints
                # Again, assert it is indeed an infeasible problem and not unbounded (in which case value is -inf).
                # (The dynamical constraints keep the problem bounded.)
                assert relaxed_prob.value >= 0.0

                # Solve relaxed problem (non-collision constraint -> non-collision objective)
                relaxed_prob = cp.Problem(cp.Minimize(obj + self.nc_weight * nc_obj + self.nc_relax_weight * nc_relax_obj),
                                          dyn_constraints + track_constraints)
                relaxed_prob.solve()

                assert not np.isinf(relaxed_prob.value)

        return p.value

    def iterative_br(self, i_ego, state, n_game_iterations=2, n_sqp_iterations=2):
        trajectories = [
            self.init_trajectory(i, state[i, :]) for i in [0, 1]
        ]
        for i_game in range(n_game_iterations - 1):
            for i in [i_ego, (i_ego + 1) % 2]:
                for i_sqp in range(n_sqp_iterations):
                    trajectories[i] = self.best_response(i, state, trajectories)

        # One last time for i_ego
        for i_sqp in range(n_sqp_iterations):
            trajectories[i_ego] = self.best_response(i_ego, state, trajectories)

        return trajectories[i_ego]

    def truncate(self, p_i, trajectory):
        """
        Truncates the trajectory at time k, so that the next point is 'ahead' of p_i.
        A point p is ahead of p_i, if p-p_i projected onto the track tangent is positive
        :param p_i: The position of the drone
        :param trajectory: The trajectory to be truncated
        :return: k, the index of the first point ahead of p_i
        """
        _, _, t, _, _, _ = self.track.track_frame_at(p_i)
        for k in range(self.n_steps):
            if t.dot(trajectory[k, :] - p_i) > 0.0:
                return k, t
        return self.n_steps, t
