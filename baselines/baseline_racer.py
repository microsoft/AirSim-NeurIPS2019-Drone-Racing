from argparse import ArgumentParser
import airsimneurips as airsim
import time
import utils

# drone_name should match the name in ~/Document/AirSim/settings.json
class BaselineRacer(object):
    def __init__(self, drone_name = "drone_1", plot_transform=True, viz_traj=True):
        self.drone_name = drone_name
        self.gate_poses_ground_truth = None
        self.plot_transform = plot_transform
        self.viz_traj = viz_traj
        self.airsim_client = airsim.MultirotorClient()
        self.airsim_client.confirmConnection()
        self.level_name = None
    
    def load_level(self, level_name, sleep_sec = 2.0):
        self.level_name = level_name
        self.airsim_client.simLoadLevel(self.level_name)
        self.airsim_client.confirmConnection() # failsafe
        time.sleep(sleep_sec) # let the environment load completely

    def initialize_drone(self):
        self.airsim_client.enableApiControl(vehicle_name=self.drone_name)
        self.airsim_client.arm(vehicle_name=self.drone_name)

        # set default values for trajectory tracker gains 
        traj_tracker_gains = airsim.TrajectoryTrackerGains(kp_cross_track = 5.0, kd_cross_track = 0.0, 
                                                            kp_vel_cross_track = 3.0, kd_vel_cross_track = 0.0, 
                                                            kp_along_track = 0.4, kd_along_track = 0.0, 
                                                            kp_vel_along_track = 0.04, kd_vel_along_track = 0.0, 
                                                            kp_z_track = 2.0, kd_z_track = 0.0, 
                                                            kp_vel_z = 0.4, kd_vel_z = 0.0, 
                                                            kp_yaw = 3.0, kd_yaw = 0.1)

        self.airsim_client.setTrajectoryTrackerGains(traj_tracker_gains.to_list(), vehicle_name=self.drone_name)
        time.sleep(0.2)

    def takeoffAsync(self):
        self.airsim_client.takeoffAsync().join()

    def takeoff_with_moveOnSpline(self, takeoff_height = 0.1):
        if self.level_name == "ZhangJiaJie_Medium":
            takeoff_height = 0.3
        takeoff_waypoint = airsim.Vector3r(0, 0, -takeoff_height)
        if(self.plot_transform):
            self.airsim_client.plot_transform([airsim.Pose(takeoff_waypoint, airsim.Quaternionr())], vehicle_name=self.drone_name)

        self.airsim_client.moveOnSplineAsync([takeoff_waypoint], vel_max=15.0, acc_max=5.0, add_curr_odom_position_constraint=True, 
            add_curr_odom_velocity_constraint=False, viz_traj=self.viz_traj, vehicle_name=self.drone_name).join()

    def get_ground_truth_gate_poses(self):
        gate_names_sorted_bad = sorted(self.airsim_client.simListSceneObjects("Gate.*"))
        # gate_names_sorted_bad is of the form `GateN_GARBAGE`. for example:
        # ['Gate0', 'Gate10_21', 'Gate11_23', 'Gate1_3', 'Gate2_5', 'Gate3_7', 'Gate4_9', 'Gate5_11', 'Gate6_13', 'Gate7_15', 'Gate8_17', 'Gate9_19']
        # we sort them by their ibdex of occurence along the race track(N), and ignore the unreal garbage number after the underscore(GARBAGE)
        gate_indices_bad = [int(gate_name.split('_')[0][4:]) for gate_name in gate_names_sorted_bad]
        gate_indices_correct = sorted(range(len(gate_indices_bad)), key=lambda k:gate_indices_bad[k])
        gate_names_sorted = [gate_names_sorted_bad[gate_idx] for gate_idx in gate_indices_correct]
        self.gate_poses_ground_truth = [self.airsim_client.simGetObjectPose(gate_name) for gate_name in gate_names_sorted]

    # scale of the vector dictates speed of the velocity constraint
    def get_gate_facing_vector_from_quaternion(self, airsim_quat, scale = 1.0):
        import numpy as np
        # convert gate quaternion to rotation matrix. 
        # ref: https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion; https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        q = np.array([airsim_quat.w_val, airsim_quat.x_val, airsim_quat.y_val, airsim_quat.z_val], dtype=np.float64)
        n = np.dot(q, q)
        if n < np.finfo(float).eps:
            return airsim.Vector3r(0.0, 1.0, 0.0)
        q *= np.sqrt(2.0 / n)
        q = np.outer(q, q)
        rotation_matrix = np.array([[1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
                                    [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
                                    [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])
        gate_facing_vector = rotation_matrix[:,1]
        return airsim.Vector3r(scale * gate_facing_vector[0], scale * gate_facing_vector[1], scale * gate_facing_vector[2])

    def fly_through_all_gates_one_by_one_with_moveOnSpline(self):
        if self.level_name == "Building99_Hard":
            vel_max = 5.0
            acc_max = 2.0

        if self.level_name in ["Soccer_Field_Medium", "Soccer_Field_Easy", "ZhangJiaJie_Medium"] :
            vel_max = 10.0
            acc_max = 5.0

        for gate_pose in self.gate_poses_ground_truth:
            if(self.plot_transform):
                self.airsim_client.plot_transform([gate_pose], vehicle_name=self.drone_name)

            self.airsim_client.moveOnSplineAsync([gate_pose.position], vel_max=vel_max, acc_max=acc_max, 
                add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=False, viz_traj=self.viz_traj, vehicle_name=self.drone_name).join()

    def fly_through_all_gates_at_once_with_moveOnSpline(self):
        if self.level_name in ["Soccer_Field_Medium", "Soccer_Field_Easy", "ZhangJiaJie_Medium"] :
            vel_max = 30.0
            acc_max = 15.0

        if self.level_name == "Building99_Hard":
            vel_max = 4.0
            acc_max = 1.0

        if(self.plot_transform):
            self.airsim_client.plot_transform(self.gate_poses_ground_truth, vehicle_name=self.drone_name)

        self.airsim_client.moveOnSplineAsync([gate_pose.position for gate_pose in self.gate_poses_ground_truth], vel_max=30.0, acc_max=15.0, 
            add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=False, viz_traj=self.viz_traj, vehicle_name=self.drone_name).join()

    def fly_through_all_gates_one_by_one_with_moveOnSplineVelConstraints(self):
        add_curr_odom_velocity_constraint = True

        if self.level_name in ["Soccer_Field_Medium", "Soccer_Field_Easy"] :
            vel_max = 15.0
            acc_max = 3.0
            speed_through_gate = 2.5

        if self.level_name == "ZhangJiaJie_Medium":
            vel_max = 10.0
            acc_max = 3.0
            speed_through_gate = 1.0

        if self.level_name == "Building99_Hard":
            vel_max = 2.0
            acc_max = 0.5
            speed_through_gate = 0.5
            add_curr_odom_velocity_constraint = False

        for gate_pose in self.gate_poses_ground_truth:
            if(self.plot_transform):
                self.airsim_client.plot_transform([gate_pose], vehicle_name=self.drone_name)

            # scale param scales the gate facing vector by desired speed. 
            self.airsim_client.moveOnSplineVelConstraintsAsync([gate_pose.position], 
                                                    [self.get_gate_facing_vector_from_quaternion(gate_pose.orientation, scale = speed_through_gate)], 
                                                    vel_max=vel_max, acc_max=acc_max, 
                                                    add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=add_curr_odom_velocity_constraint, 
                                                    viz_traj=self.viz_traj, vehicle_name=self.drone_name).join()

    def fly_through_all_gates_at_once_with_moveOnSplineVelConstraints(self):
        if self.level_name in ["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium"]:
            vel_max = 15.0
            acc_max = 7.5
            speed_through_gate = 2.5

        if self.level_name == "Building99_Hard":
            vel_max = 5.0
            acc_max = 2.0
            speed_through_gate = 1.0

        if(self.plot_transform):
            self.airsim_client.plot_transform(self.gate_poses_ground_truth, vehicle_name=self.drone_name)

        self.airsim_client.moveOnSplineVelConstraintsAsync([gate_pose.position for gate_pose in self.gate_poses_ground_truth], 
                [self.get_gate_facing_vector_from_quaternion(gate_pose.orientation, scale = speed_through_gate) for gate_pose in self.gate_poses_ground_truth], 
                vel_max=15.0, acc_max=7.5, 
                add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, 
                viz_traj=self.viz_traj, vehicle_name=self.drone_name).join()

def main(args):
    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    baseline_racer = BaselineRacer(drone_name="drone_1", plot_transform=args.plot_transform, viz_traj=args.viz_traj)
    baseline_racer.load_level(args.level_name)
    baseline_racer.initialize_drone()
    baseline_racer.takeoff_with_moveOnSpline()
    baseline_racer.get_ground_truth_gate_poses()

    if args.planning_baseline_type == "all_gates_at_once" :
        if args.planning_and_control_api == "moveOnSpline":
            baseline_racer.fly_through_all_gates_at_once_with_moveOnSpline()
        if args.planning_and_control_api == "moveOnSplineVelConstraints":
            baseline_racer.fly_through_all_gates_at_once_with_moveOnSplineVelConstraints()

    if args.planning_baseline_type == "all_gates_one_by_one":
        if args.planning_and_control_api == "moveOnSpline":
            baseline_racer.fly_through_all_gates_one_by_one_with_moveOnSpline()
        if args.planning_and_control_api == "moveOnSplineVelConstraints":
            baseline_racer.fly_through_all_gates_one_by_one_with_moveOnSplineVelConstraints()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard"], default="ZhangJiaJie_Medium")
    parser.add_argument('--planning_baseline_type', type=str, choices=["all_gates_at_once","all_gates_one_by_one"], default="all_gates_at_once")
    parser.add_argument('--planning_and_control_api', type=str, choices=["moveOnSpline", "moveOnSplineVelConstraints"], default="moveOnSpline")
    parser.add_argument('--plot_transform', dest='plot_transform', action='store_true', default=False)
    parser.add_argument('--viz_traj', dest='viz_traj', action='store_true', default=False)
    args = parser.parse_args()
    main(args)