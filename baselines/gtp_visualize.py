import numpy as np

# Plots gate centers
def plot_gates_2d(ax, gate_poses):
    gate_pos = np.array([pose.position.to_numpy_array() for pose in gate_poses])
    ax.plot(gate_pos[:, 1], gate_pos[:, 0], 'x')

def plot_track_arrows(ax, track):
    # Plot gates
    for i in range(track.n_gates):
        ax.arrow(track.gates[i].position.y_val,
                 track.gates[i].position.x_val,
                 track.tangents[i, 1],
                 track.tangents[i, 0],
                 head_width=0.64, head_length=2.0, fc='k', ec='k')

    for i in range(len(track.track_centers)):
        ax.arrow(track.track_centers[i, 1],
                 track.track_centers[i, 0],
                 track.track_tangents[i, 1],
                 track.track_tangents[i, 0],
                 head_width=0.08, head_length=0.25, fc='k', ec='k')

def plot_track(ax, track):
    left_boundary = track.track_centers + track.track_normals*track.track_widths[:, np.newaxis]
    right_boundary = track.track_centers - track.track_normals*track.track_widths[:, np.newaxis]

    # Plot the spline
    ax.plot(track.track_centers[:, 1], track.track_centers[:, 0], '--')
    ax.plot(left_boundary[:, 1], left_boundary[:, 0], 'b-')
    ax.plot(right_boundary[:, 1], right_boundary[:, 0], 'b-')

def plot_track3d(ax, track):
    left_boundary = track.track_centers + track.track_normals*track.track_widths[:, np.newaxis] 
    right_boundary = track.track_centers - track.track_normals*track.track_widths[:, np.newaxis]
    track_verticals = np.cross(track.track_tangents, track.track_normals)
    upper_boundary = track.track_centers + track_verticals*track.track_heights[:,np.newaxis]
    lower_boundary = track.track_centers - track_verticals*track.track_heights[:,np.newaxis]

    # Plot the spline
    ax.plot(track.track_centers[:, 1], track.track_centers[:, 0], -track.track_centers[:, 2], '--')
    ax.plot(left_boundary[:, 1], left_boundary[:, 0], -track.track_centers[:, 2], 'b-')
    ax.plot(right_boundary[:, 1], right_boundary[:, 0], -track.track_centers[:, 2], 'b-')
    ax.plot(upper_boundary[:, 1], upper_boundary[:, 0], -upper_boundary[:, 2], 'g-')
    ax.plot(lower_boundary[:, 1], lower_boundary[:, 0], -lower_boundary[:, 2], 'g-')

def plot_state(ax, state):
    return ax.plot(state[:, 1], state[:, 0], 'o')

def replot_state(line, state):
    line.set_xdata(state[:, 1])
    line.set_ydata(state[:, 0])

def plot_trajectory_2d(ax, trajectory):
    return ax.plot(trajectory[:, 1], trajectory[:, 0])

def replot_trajectory_2d(line, trajectory):
    line.set_xdata(trajectory[:, 1])
    line.set_ydata(trajectory[:, 0])