from argparse import ArgumentParser
from baseline_racer import BaselineRacer
from imutils.perspective import order_points
from itertools import combinations
from numpy.linalg import multi_dot
import airsimneurips as airsim
import copy
import cv2
import math
import numpy as np
import time

class BaselineRacerPerception(BaselineRacer):
    def __init__(self, drone_name = "drone_1",  viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0], viz_image_cv2=True):
        BaselineRacer.__init__(self, drone_name, viz_traj, viz_traj_color_rgba, viz_image_cv2)
        self.eps = 0.01
        self.kernel = np.ones((3,3),np.uint8)
        self.gate_corners_flat = np.float32([[[94, 64],[204, 64],[204, 174], [94, 174]]])
        self.camera_matrix = np.array([[160.000, 0.000000, 160.000], [0.000000, 160.000, 120.000], [0.000000, 0.000000, 1.000000]])
        self.gate_center_pixel_flat = np.array([149,119,1]).T
        self.no_gate_count = 0
        self.measurement_count = 0
        self.lower_green1 = np.array([0, 120, 0])
        self.upper_green1 = np.array([50, 255, 50])
        self.lower_green2 = np.array([0, 175, 0])
        self.upper_green2 = np.array([170, 255, 150])
        self.dist_coeff = np.zeros((1,5))
        self.waypoint_gate_1 = np.array([0.0, 0.0, 0.0, 1.0]).T.reshape(4,1)
        self.waypoint_gate_2 = np.array([0.0, 0.0, -4.0, 1.0]).T.reshape(4,1)
        self.gate_points_3D = 1.5*np.array([[0,0,0],[-1.0,1.0,0],[1.0,1.0,0],[1.0,-1.0,0],[-1.0,-1.0,0]])
        self.C = np.identity(3)
        self.Q = 50*np.identity(3)
        self.too_close_count = 0
        self.largest_next_gate_position_dist = 1.7
        self.aspect_ratio_max = 2.0

    def get_rotation_matrix_quad_frame_to_global_frame(self,state):
        q0 = state.orientation.w_val
        q1 = state.orientation.x_val
        q2 = state.orientation.y_val
        q3 = state.orientation.z_val
        rot_matrix_quad2global = np.array([[1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3-q0*q2)],
                                        [2*(q1*q2 + q0*q3), 1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)],
                                        [2*(q1*q3-q0*q2), 2*(q1*q0+q2*q3), 1-2*(q1**2+q2**2)]])
        return rot_matrix_quad2global

    def find_aspect_ratio(self, gate_corners):
        aspect_ratio_mtx = np.empty((1,4))
        aspect_ratio_mtx[0,0] = abs(np.float32(gate_corners[1][1] - gate_corners[2][1])/np.float32(gate_corners[0][0] - gate_corners[1][0]+self.eps))
        aspect_ratio_mtx[0,1] = abs(np.float32(gate_corners[0][1] - gate_corners[3][1])/np.float32(gate_corners[2][0] - gate_corners[3][0]+self.eps))
        aspect_ratio_mtx[0,2] = abs(np.float32(gate_corners[0][1] - gate_corners[3][1])/np.float32(gate_corners[0][0] - gate_corners[1][0]+self.eps))
        aspect_ratio_mtx[0,3] = abs(np.float32(gate_corners[1][1] - gate_corners[2][1])/np.float32(gate_corners[2][0] - gate_corners[3][0]+self.eps))
        large_aspect_ratio = 1.0
        for i in range(4):
            if aspect_ratio_mtx[0,i] < 1.0:
                aspect_ratio_mtx[0,i] = 1/aspect_ratio_mtx[0,i]
            if aspect_ratio_mtx[0,i] > self.aspect_ratio_max:
                large_aspect_ratio = aspect_ratio_mtx[0,i]
        return large_aspect_ratio

    def gate_frame_waypoint_to_global_waypoint(self,state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,waypoint_gate):
        waypoint_rel = np.dot(rot_translation_matrix_gate2quad, waypoint_gate)
        waypoint_rel = np.dot(rot_matrix_quad2global, np.array([waypoint_rel[2], waypoint_rel[0], -waypoint_rel[1]]))
        waypoint_glob = np.reshape(np.array([state.position.x_val + waypoint_rel[0], state.position.y_val + waypoint_rel[1], state.position.z_val - waypoint_rel[2]]),(1,3))
        return waypoint_glob

    def find_measurement_error(self,four_gate_corners, mu_1, state, rot_matrix_quad2global):
        four_gate_corners = order_points(four_gate_corners.reshape(4,2))
        gate_corners_plot = copy.deepcopy(four_gate_corners)
        four_gate_corners = np.float32(four_gate_corners).reshape(-1,1,2)

        # find homography matrix between gate corner points in baseline image and corner points in image taken
        homography_matrix, __ = cv2.findHomography(four_gate_corners, self.gate_corners_flat, cv2.RANSAC, 5.0)

        # find middle of gate pixel coordinates
        gate_center_pixel = np.dot(np.linalg.inv(homography_matrix), self.gate_center_pixel_flat)
        gate_center_pixel = (int(gate_center_pixel[0]/(gate_center_pixel[2]+self.eps)), int(gate_center_pixel[1]/(gate_center_pixel[2]+self.eps)))

        # coordinates of gate corners in image (pixel coordinates)
        gate_points_2D = np.float32(np.concatenate((np.array(gate_center_pixel).reshape(-1,1,2),four_gate_corners),axis=0))

    	# find rotation and translation from gate frame to quad frame
        __, rvec, tvec = cv2.solvePnP(self.gate_points_3D,gate_points_2D,self.camera_matrix,self.dist_coeff)
        rvec_full, __ = cv2.Rodrigues(rvec)
        rot_translation_matrix_gate2quad = np.concatenate((rvec_full, tvec), axis=1)
        rot_translation_matrix_gate2quad = np.concatenate((rot_translation_matrix_gate2quad, np.array([0,0,0,1]).reshape(1,4)), axis=0)

        # gate frame waypoint to global frame waypoint
        waypoint_glob_1 = self.gate_frame_waypoint_to_global_waypoint(state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,self.waypoint_gate_1)
        waypoint_glob_2 = self.gate_frame_waypoint_to_global_waypoint(state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,self.waypoint_gate_2)
        waypoint_1_error = mu_1 - np.reshape(waypoint_glob_1,(3,1))
        measurement_error = np.linalg.norm(waypoint_1_error)
        return measurement_error, waypoint_glob_1, waypoint_glob_2, gate_center_pixel, gate_corners_plot

    def kalman_filter(self, mu, sigma, y):
        K = multi_dot([sigma,self.C.T,np.linalg.inv(multi_dot([self.C,sigma,self.C.T])+self.Q)])
        mu = mu + np.dot(K,(y-np.dot(self.C,mu)))
        sigma = np.dot((np.identity(3)-np.dot(K,self.C)),sigma)
        return mu, sigma

    def run(self):
        # Move through first gate
        self.get_ground_truth_gate_poses() # noisy since tier 2 is default
        next_gate_idx = 0
        mu_1_airsimvector = self.gate_poses_ground_truth[0].position
        mu_1 = np.reshape(np.array([mu_1_airsimvector.x_val, mu_1_airsimvector.y_val, mu_1_airsimvector.z_val]), (3,1))
        sigma_1 = 50*np.identity(3)
        sigma_2 = 50*np.identity(3)
        while self.airsim_client.isApiControlEnabled(vehicle_name=self.drone_name):
        	# Take image
            response = self.airsim_client.simGetImages([airsim.ImageRequest("fpv_cam", airsim.ImageType.Scene, False, False)])
            img1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8) # get numpy array
            image_rgb = img1d.reshape(response[0].height, response[0].width, 3)

        	# Get quad state when the image was taken
            state = self.airsim_client.simGetVehiclePose()
            rot_matrix_quad2global = self.get_rotation_matrix_quad_frame_to_global_frame(state)
            mask1 = cv2.inRange(image_rgb,self.lower_green1,self.upper_green1)
            mask2 = cv2.inRange(image_rgb,self.lower_green2,self.upper_green2)
            mask = mask1 + mask2
            dilated_gate = cv2.dilate(mask,self.kernel, iterations=8)
            #eroded_gate = cv2.erode(dilated_gate,self.kernel, iterations=8)
            gate_contours = None
            gate_contours, hierarchy = cv2.findContours(dilated_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            cv2.imshow("mask", mask)
            cv2.imshow("dilated_gate", dilated_gate)
            #cv2.imshow("eroded_gate", eroded_gate)
            cv2.waitKey(1)

            largest_area = 0.0
            gate_corners = None
            smallest_measurement_error = 100
            #check for each contour
            for contour in gate_contours:
                gate_contour_epsilon = 0.01 * cv2.arcLength(contour, True)
                gate_contour_approx = cv2.approxPolyDP(contour,gate_contour_epsilon,True)
                gate_corners = cv2.convexHull(gate_contour_approx)
                gate_corner_combinations = [None]
                gate_corner_combinations = list(combinations(gate_corners,4))
                # check for each combination of four gate corner points within each contour
                for comb in gate_corner_combinations:
                    four_gate_corners = np.empty((0,2))
                    for i in comb:
                        i = np.reshape(i,(1,2))
                        four_gate_corners = np.append(four_gate_corners,i,axis=0)
                    #now find the measurement error
                    measurement_error, waypoint_glob_1, waypoint_glob_2, gate_center_pixel, gate_corners_plot  = self.find_measurement_error(four_gate_corners, mu_1, state, rot_matrix_quad2global)
                    aspect_ratio = self.find_aspect_ratio(four_gate_corners)
                    if measurement_error < smallest_measurement_error and aspect_ratio < self.aspect_ratio_max:
                        waypoint_1_best = copy.deepcopy(waypoint_glob_1)
                        waypoint_2_best = copy.deepcopy(waypoint_glob_2)
                        smallest_measurement_error = copy.deepcopy(measurement_error)
                        gate_center_pixel_best = copy.deepcopy(gate_center_pixel)
                        gate_corners_plot_best = copy.deepcopy(gate_corners_plot)
            next_gate_position_dist = abs(np.linalg.norm(np.reshape(np.array([state.position.x_val,state.position.y_val,state.position.z_val]),(3,1)) - mu_1))
            #print(smallest_measurement_error, next_gate_position_dist, next_gate_idx)

            # MOVE
            if smallest_measurement_error > 5.0:
                self.no_gate_count += 1
                print("Gate NOT detected")
                if self.no_gate_count == 10 and next_gate_position_dist >= self.largest_next_gate_position_dist:
                    self.no_gate_count = 0
                    self.too_close_count = 0
                    print("Move toward best estimate")
                    self.airsim_client.moveOnSplineAsync([airsim.Vector3r(mu_1[0,0],mu_1[1,0], mu_1[2,0])], vel_max=15.0, acc_max=4.0, add_position_constraint=True, add_velocity_constraint=False,
                        add_acceleration_constraint=False, viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, vehicle_name=self.drone_name, replan_from_lookahead=True)
                elif next_gate_position_dist < self.largest_next_gate_position_dist: #don't collect measurements
                    print("Gate too close")
                    self.no_gate_count = 0
                    self.too_close_count += 1
                    if self.too_close_count == 1:
                        print("reset gate too close")
                        next_gate_idx += 1
                        if next_gate_idx >= len(self.gate_poses_ground_truth):
                            next_gate_idx = 0
                            print("race end")
                        mu_1_airsimvector = self.gate_poses_ground_truth[next_gate_idx].position
                        mu_1 = np.reshape(np.array([mu_1_airsimvector.x_val, mu_1_airsimvector.y_val, mu_1_airsimvector.z_val]), (3,1))
                        sigma_1 = 50*np.identity(3)
                        sigma_2 = 50*np.identity(3)
                        self.measurement_count = 0
                        self.too_close_count = 0
            else:
                self.no_gate_count = 0
                self.measurement_count += 1
                if self.measurement_count == 1:
                    mu_2 = np.reshape(waypoint_2_best,(3,1)) #initialize mu_2
                # filter both waypoints
                mu_1, sigma_1 = self.kalman_filter(mu_1, sigma_1, np.reshape(waypoint_1_best,(3,1)))
                mu_2, sigma_2 = self.kalman_filter(mu_2, sigma_2, np.reshape(waypoint_2_best,(3,1)))
                waypoint_1 = airsim.Vector3r(mu_1[0,0],mu_1[1,0], mu_1[2,0])
                waypoint_2 = airsim.Vector3r(mu_2[0,0],mu_2[1,0], mu_2[2,0])

                if next_gate_position_dist < self.largest_next_gate_position_dist: #don't collect measurements, continue to waypoint, reset filter with new mu and sigmas
                    print("Gate too close")
                    time.sleep(3)
                    self.too_close_count += 1
                    if self.too_close_count == 1:
                        print("reset gate too close")
                        next_gate_idx += 1
                        if next_gate_idx >= len(self.gate_poses_ground_truth):
                            next_gate_idx = 0
                            print("race end")
                        mu_1_airsimvector = self.gate_poses_ground_truth[next_gate_idx].position
                        mu_1 = np.reshape(np.array([mu_1_airsimvector.x_val, mu_1_airsimvector.y_val, mu_1_airsimvector.z_val]), (3,1))
                        sigma_1 = 50*np.identity(3)
                        sigma_2 = 50*np.identity(3)
                        self.measurement_count = 0
                else:
                    print("Gate detected, Measurement Taken")
                    print(self.measurement_count)
                    self.too_close_count = 0
                    cv2.circle(image_rgb, (int(gate_center_pixel_best[0]), int(gate_center_pixel_best[1])), 10, (255, 0, 0), -1)
                    cv2.circle(image_rgb, (int(gate_corners_plot_best[0][0]), int(gate_corners_plot_best[0][1])), 10, (255, 100, 0), -1)
                    cv2.circle(image_rgb, (int(gate_corners_plot_best[1][0]), int(gate_corners_plot_best[1][1])), 10, (255, 0, 100), -1)
                    cv2.circle(image_rgb, (int(gate_corners_plot_best[2][0]), int(gate_corners_plot_best[2][1])), 10, (255, 200, 0), -1)
                    cv2.circle(image_rgb, (int(gate_corners_plot_best[3][0]), int(gate_corners_plot_best[3][1])), 10, (255, 0, 200), -1)
                    cv2.imshow("image_rgb", image_rgb)
                    cv2.waitKey(1)
                    self.airsim_client.moveOnSplineAsync([waypoint_1,waypoint_2], vel_max=5.0, acc_max=3.0, add_position_constraint=True, add_velocity_constraint=False,
                        add_acceleration_constraint=False, viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, vehicle_name=self.drone_name, replan_from_lookahead=True)

def main(args):
    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    baseline_racer = BaselineRacerPerception(drone_name="drone_1", viz_traj=args.viz_traj, viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0], viz_image_cv2=args.viz_image_cv2)
    baseline_racer.load_level(args.level_name)
    baseline_racer.start_race(args.race_tier)
    baseline_racer.initialize_drone()
    baseline_racer.takeoff_with_moveOnSpline()
    baseline_racer.run()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard", "Qualifier_Tier_1", "Qualifier_Tier_2", "Qualifier_Tier_3"], default="Qualifier_Tier_2")
    parser.add_argument('--viz_traj', dest='viz_traj', action='store_true', default=False)
    parser.add_argument('--enable_viz_image_cv2', dest='viz_image_cv2', action='store_true', default=False)
    parser.add_argument('--race_tier', type=int, choices=[1,2,3], default=2)
    args = parser.parse_args()
    main(args)
