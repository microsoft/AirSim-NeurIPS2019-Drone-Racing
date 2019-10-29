import gtp
from baseline_racer import BaselineRacer
from utils import to_airsim_vector, to_airsim_vectors
from visualize import *

# Use non interactive matplotlib backend
import matplotlib
# matplotlib.use('Agg')
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3d plotting

import airsimneurips as airsim
import time
import numpy as np

import argparse


class BaselineRacerGTP(BaselineRacer):
    def __init__(self, traj_params, drone_names, drone_i, drone_params,
                 use_vel_constraints=False,
                 plot_gtp=False):
        super().__init__(drone_name=drone_names[drone_i], 
          # plot_transform=True, 
          viz_traj=True)
        self.drone_names = drone_names
        self.drone_i = drone_i
        self.drone_params = drone_params
        self.traj_params = traj_params

        self.use_vel_constraints = use_vel_constraints
        self.plot_gtp = plot_gtp

        self.controller = None

        # For plotting: Just some fig, ax and line objects to keep track of
        if self.plot_gtp:
            self.fig, self.ax = plt.subplots()
            self.line_state = None
            self.lines = [None] * 2
            
            # plot 3d track just once for visualization
            self.fig2 = plt.figure(2)
            self.ax3d = self.fig2.add_subplot(111, projection='3d')

        print('baseline_racer_gtp {} ready!'.format(self.drone_name))
        self.pause_counter = 0
        self.pause_limit = 3
        self.send_new_traj = True
        self.image_num = 0

    def update_and_plan(self):
        # Retrieve the current state from AirSim
        position_airsim = []
        for drone_name in self.drone_names:
            position_airsim.append(self.airsim_client.simGetObjectPose(drone_name).position)

        state = np.array([position.to_numpy_array() for position in position_airsim])

        if self.plot_gtp:
            # Plot or update the state
            if self.line_state is None:
                self.line_state, = plot_state(self.ax, state)
            else:
                replot_state(self.line_state, state)

        trajectory = self.controller.iterative_br(self.drone_i, state)
        # trajectory = self.controller.init_trajectory(0, state[0,:])

        # Now, let's issue the new trajectory to the trajectory planner
        # Fetch the current state first, to see, if our trajectory is still planned for ahead of us
        new_state_i = self.airsim_client.simGetObjectPose(self. drone_name).position.to_numpy_array()

        if self.plot_gtp:
            replot_state(self.line_state, state)

        # As we move while computing the trajectory,
        # make sure that we only issue the part of the trajectory, that is still ahead of us
        k_truncate, _ = self.controller.truncate(new_state_i, trajectory[:, :])
        print("k_truncate: ", k_truncate)

        # k_truncate == args.n means that the whole trajectory is behind us, and we only issue the last point
        if k_truncate == self.traj_params.n:
            k_truncate = self.traj_params.n - 1
            print('DEBUG: truncating entire trajectory, k_truncate = {}'.format(k_truncate))

        if self.plot_gtp:
            # For our 2D trajectory, let's plot or update
            if self.lines[self.drone_i] is None:
                self.lines[self.drone_i], = plot_trajectory_2d(self.ax, trajectory[k_truncate:, :])
            else:
                replot_trajectory_2d(self.lines[self.drone_i], trajectory[k_truncate:, :])

        # Finally issue the command to AirSim.
        if not self.use_vel_constraints:
            # This returns a future, that we do not call .join() on, as we want to re-issue a new command   
            # once we compute the next iteration of our high-level planner
            
            # if self.send_new_traj:
            #     print('Sending new trajectory!')
            #     self.send_new_traj = False
            print('Final position trajectory: ', trajectory[k_truncate:, :])

            self.airsim_client.moveOnSplineAsync(
                to_airsim_vectors(trajectory[k_truncate:, :]),
                add_position_constraint=False,
                add_velocity_constraint=True,
                vel_max=self.drone_params[self.drone_i]["v_max"],
                acc_max=self.drone_params[self.drone_i]["a_max"],
                viz_traj=self.viz_traj, 
                vehicle_name=self.drone_name,
                replan_from_lookahead=False,
                replan_lookahead_sec=0.0)
        else:
            # Compute the velocity as the difference between waypoints
            vel_constraints = np.zeros_like(trajectory[k_truncate:, :])
            vel_constraints[1:, :] = trajectory[k_truncate + 1:, :] - trajectory[k_truncate:-1, :]
            # If we use the whole trajectory, the velocity constraint at the first point
            # is computed using the current position
            if k_truncate == 0:
                vel_constraints[0, :] = trajectory[k_truncate, :] - new_state_i
            else:
                vel_constraints[0, :] = trajectory[k_truncate, :] - trajectory[k_truncate - 1, :]

            # vel_constraints /= self.traj_params.dt
            # vel_constraints /= 100.0
            print('Final position trajectory: ', trajectory[k_truncate:, :])
            print('Final velocity trajectory: ', vel_constraints)
            
            self.airsim_client.moveOnSplineVelConstraintsAsync(
                to_airsim_vectors(trajectory[k_truncate:, :]),
                to_airsim_vectors(vel_constraints),
                add_position_constraint=True,
                add_velocity_constraint=True,
                vel_max=self.drone_params[self.drone_i]["v_max"],
                acc_max=self.drone_params[self.drone_i]["a_max"],
                viz_traj=self.viz_traj,
                vehicle_name=self.drone_name)

        if self.plot_gtp:
            # Refresh the updated plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            # # Save figure
            # figure_file_name = '/home/ericcristofalo/Documents/game_of_drones/log/image_' + str(self.image_num) + '.png'
            # print('saving image: ', figure_file_name)
            # self.fig.savefig(figure_file_name)
            # self.image_num = self.image_num + 1
            # # Set the figure bounds around drone i for visualization
            # self.fig.xlim(state[self.drone_i][1] - 10, state[self.drone_i][1] + 10)
            # self.fig.ylim(state[self.drone_i][0] - 10, state[self.drone_i][0] + 10)

        # # Counter for pause debugging
        # # self.pause_counter = self.pause_counter + 1
        # # if (self.pause_counter == 9):
        # #     self.pause_counter = 0;
        # #     self.send_new_traj = True
        # self.pause_counter += 1
        # if (self.pause_counter == self.pause_limit):
        #     airsim_pause_check = self.airsim_client.simIsPause()
        #     if airsim_pause_check:  # if simulation is paused
        #         self.airsim_client.simPause(False)  # unpause
        #         self.pause_limit = 3  # run for this many steps
        #     else:
        #         self.airsim_client.simPause(True)  # pause
        #         self.pause_limit = 3  # pause for this many steps
        #     self.pause_counter = 0

    def run(self):
        self.get_ground_truth_gate_poses()

        # We pretend we have two different controllers for the drones,
        # so let's instantiate  two
        self.controller = gtp.IBRController(self.traj_params, self.drone_params, self.gate_poses_ground_truth)

        if self.plot_gtp:
            # Let's plot the gates, and the fitted track.
            plot_gates_2d(self.ax, self.gate_poses_ground_truth)
            plot_track(self.ax, self.controller.track)
            plot_track_arrows(self.ax, self.controller.track)
            plt.ion()  # turn on interactive mode to plot while running script
            self.fig.show()

            plot_track3d(self.ax3d, self.controller.track)
            self.fig2.show()


        while self.airsim_client.isApiControlEnabled(vehicle_name=self.drone_name):
            self.update_and_plan()


def main(args):
    drone_names = ["drone_1", "drone_2"]
    drone_params = [
        {"r_safe": 0.5,
         "r_coll": 0.5,
         "v_max": 80.0,
         "a_max": 30.0},
        {"r_safe": 0.4,
         "r_coll": 0.3,
         "v_max": 20.0,
         "a_max": 10.0}]

    # set drone dt as function of max speed
    # designed to be 0.1s for v_max=20m/s
    # args.dt = 1.0/drone_params[0]["v_max"]/1.0

    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    baseline_racer_gtp = BaselineRacerGTP(
        traj_params=args,
        drone_names=drone_names,
        drone_i=0,  # index of the first drone
        drone_params=drone_params,
        use_vel_constraints=args.vel_constraints, 
        plot_gtp=args.plot_gtp)

    # baseline_racer_opp = BaselineRacer(drone_name=drone_names[1], plot_transform=True, viz_traj=True)
    # baseline_racer_opp = BaselineRacer(drone_name=drone_names[1], viz_traj=args.viz_traj, viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0], viz_image_cv2=False)

    baseline_racer_gtp.load_level(args.level_name)
    baseline_racer_gtp.start_race(args.race_tier)

    baseline_racer_gtp.initialize_drone()
    # baseline_racer_opp.initialize_drone()

    baseline_racer_gtp.takeoff_with_moveOnSpline()
    # baseline_racer_opp.takeoff_with_moveOnSpline()

    # baseline_racer_opp.get_ground_truth_gate_poses()
    # baseline_racer_opp.fly_through_all_gates_at_once_with_moveOnSpline()

    # give opponent a little advantage
    time.sleep(0.0)
    baseline_racer_gtp.run()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--dt', type=float, default=0.05)
    parser.add_argument('--n', type=int, default=12)
    parser.add_argument('--vel_constraints', dest='vel_constraints', action='store_true', default=False)
    parser.add_argument('--plot_gtp', dest='plot_gtp', action='store_true', default=False)
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard", 
        "Qualifier_Tier_1", "Qualifier_Tier_2", "Qualifier_Tier_3"], default="ZhangJiaJie_Medium")
    parser.add_argument('--enable_viz_traj', dest='viz_traj', action='store_true', default=False)
    parser.add_argument('--race_tier', type=int, choices=[1,2,3], default=1)
    args = parser.parse_args()
    main(args)