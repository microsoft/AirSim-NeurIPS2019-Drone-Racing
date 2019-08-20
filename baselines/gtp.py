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
    # v_rotated_ = v_.rotate(q)  # seems like it's a rotate by operator
    return airsim.Vector3r(x_val=v_rotated_.x_val, y_val=v_rotated_.y_val, z_val=v_rotated_.z_val)


"""
This class represents a Track defined by Gates.
A spline is fitted through the Gates with tangential constraints.
This is then resampled to give smooth tangents and normals  
"""
class SplinedTrack:
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

    # Find closest re-sampled point
    # In theory, could find closest point on Spline directly,
    # see e.g. https://computingandrecording.wordpress.com/2017/03/20/closest-point-to-a-cubic-spline/
    # returns idx of point, center, tangent and normal
    def track_frame_at(self, p):
        i = np.linalg.norm(self.track_centers - p, axis=1).argmin()
        return i, self.track_centers[i], self.track_tangents[i], self.track_normals[i], \
               self.track_widths[i], self.track_heights[i]


# Every participant must supply a class called Controller
class Controller:
    def __init__(self, params, drone_params, gate_poses):
        self.dt = params.dt
        self.n_steps = params.n
        self.drone_params = drone_params

        self.track = SplinedTrack(gate_poses)

    # Based on the start position p_0, return an initial guess for a trajectory
    # This is simply a line following the tangent
    def init_trajectory(self, i_0, p_0):
        N = self.n_steps
        v_ego = self.drone_params[i_0]["v_max"]
        trajectory = np.zeros(shape=(N, 3))
        p = np.copy(p_0)  # Copy state as it gets modified throughout the loop
        for i in range(N):
            idx, c, t, n, width, height = self.track.track_frame_at(p)
            p += self.dt * v_ego * t
            trajectory[i, :] = p
        return trajectory

    # Based on current trajectories, compute the best-response (BR) of player i,
    # and return it.
    def best_response(self, i_0, state, trajectories):
        i_1 = (i_0 + 1) % 2
        v_ego = self.drone_params[i_0]["v_max"]
        r_coll_ego = self.drone_params[i_0]["r_coll"]
        r_coll_opp = self.drone_params[i_1]["r_coll"]
        r_safe_ego = self.drone_params[i_0]["r_safe"]
        r_safe_opp = self.drone_params[i_1]["r_safe"]

        # These are some parameters that could be tuned.
        # They control how the safety penalty and the relaxed constraints are weighted.
        nc_weight = 2.0
        nc_relax_weight = 128.0
        track_relax_weight = 16.0

        N = self.n_steps
        p = cp.Variable(shape=(N, 3))

        # Dynamical Constraints:
        # ||p_0 - p[0]|| <= v*dt
        init_dyn_constraint = cp.SOC(cp.Constant(v_ego * self.dt), cp.Constant(state[i_0, :]) - p[0, :])
        # dyn_constraints = []
        # ||p[k+1] - p[k]|| <= v*dt
        dyn_constraints = [init_dyn_constraint] + [
            cp.SOC(cp.Constant(v_ego * self.dt), p[k + 1, :] - p[k, :]) for k in range(N - 1)]

        track_constraints = []
        track_obj = cp.Constant(0)
        # exponentially decreasing weight
        track_objective_exp = 0.5

        for k in range(N):
            idx, c, t, n, width, height = \
                self.track.track_frame_at(trajectories[i_0][k, :])
            track_constraints.append(n.T @ p[k, :] - n.dot(c) <= width - r_coll_ego)
            track_constraints.append(n.T @ p[k, :] - n.dot(c) >= -(width - r_coll_ego))
            # 3D Height. This assumes non rotated gates.
            track_constraints.append(p[k, 2] - c[2] <= height - r_coll_ego)
            track_constraints.append(p[k, 2] - c[2] >= -(height - r_coll_ego))

            track_obj += (track_objective_exp ** k) * (
                    cp.pos(n.T @ p[k, :] - n.dot(c) - (width - r_coll_ego)) +
                    cp.pos(-(n.T @ p[k, :] - n.dot(c) + (width - r_coll_ego))))

        non_collision_constraints = []
        nc_obj = cp.Constant(0)
        nc_relax_obj = cp.Constant(0)
        # exponentially decreasing weight
        non_collision_objective_exp = 0.5

        for k in range(N):
            p_opp = trajectories[i_1][k, :]
            p_ego = trajectories[i_0][k, :]
            beta = p_opp - p_ego
            assert np.linalg.norm(beta) >= 1e-6
            beta /= np.linalg.norm(beta)
            #     n.T * (p_opp - p_ego) >= r_coll_opp + r_coll_ego
            non_collision_constraints.append(beta.dot(p_opp) - beta.T @ p[k, :] >= r_coll_opp + r_coll_ego)
            # <=> -n.T * p_ego >= r_coll_opp + r_coll_ego - n.T * p_opp
            # <=> n.T * p_ego <= n.T * p_opp - (r_coll_opp + r_coll_ego)

            # For objective use safety radii
            nc_obj += (non_collision_objective_exp ** k) * cp.pos(
                (r_safe_opp + r_safe_ego) - (beta.dot(p_opp) - beta.T @ p[k, :]))

            nc_relax_obj += (non_collision_objective_exp ** k) * cp.pos(
                (r_coll_opp + r_coll_ego) - (beta.dot(p_opp) - beta.T @ p[k, :]))

        _, _, t, _, _, _ = self.track.track_frame_at(trajectories[i_0][-1, :])
        obj = -t.T @ p[-1, :]
        prob = cp.Problem(cp.Minimize(obj + nc_weight * nc_obj),
                          dyn_constraints + track_constraints + non_collision_constraints)
        prob.solve()

        if np.isinf(prob.value):
            print("Relaxing track constraints")
            # If still not feasible, relax track constraints
            # It has to be an infeasible problem, as the dynamical constraints keep it bounded
            assert prob.value >= 0.0

            # Solve relaxed problem (non-collision constraint -> non-collision objective)
            relaxed_prob = cp.Problem(cp.Minimize(obj + nc_weight * nc_obj + track_relax_weight * track_obj),
                                      dyn_constraints + non_collision_constraints)
            relaxed_prob.solve()

            if np.isinf(relaxed_prob.value):
                print("Relaxing non collision constraints")
                # There's a problem with solving the system
                # It has to be an infeasible problem, as the dynamical constraints keep it bounded
                assert prob.value >= 0.0

                # Solve relaxed problem (non-collision constraint -> non-collision objective)
                relaxed_prob = cp.Problem(cp.Minimize(obj + nc_weight * nc_obj + nc_relax_weight * nc_relax_obj),
                                          dyn_constraints + track_constraints)
                relaxed_prob.solve()

                assert not np.isinf(relaxed_prob.value)

        return p.value

    def iterative_br(self, i_ego, state, n_game_iterations, n_sqp_iterations):
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

        return trajectories

    def callback(self, i_0, state, images):
        # For Tier I, images is simply empty, so let's ignore that
        # Use state of both players to plan
        # Assume other player goes
        trajectories = self.iterative_br(i_0, state, 2, 3)
        return trajectories[i_0]

    # If trajectory at time k projected onto the track tangent is ahead of state_i, return k
    def truncate(self, i, state_i, trajectory):
        _, _, t, _, _, _ = self.track.track_frame_at(state_i)
        for k in range(self.n_steps):
            if t.dot(trajectory[k, :] - state_i) > 0.0:
                return k, t
        return self.n_steps, t
