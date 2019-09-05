from argparse import ArgumentParser
from baseline_racer import BaselineRacer
from imutils.perspective import order_points
from itertools import combinations
import airsimneurips as airsim
import copy
import cv2
import math
import numpy as np
import time

class BaselineRacerPerception(BaselineRacer):
    #def __init__(self, drone_name = "drone_1", plot_transform=False, viz_traj=False):
    def __init__(self, drone_name = "drone_1",  viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0], viz_image_cv2=True):
        #BaselineRacer.__init__(self, drone_name, plot_transform, viz_traj)
        BaselineRacer.__init__(self, drone_name, viz_traj, viz_traj_color_rgba, viz_image_cv2)
        self.eps = 0.01
        self.aspect_ratio_max = 1.4
        self.waypoint_ave_2 = np.zeros((1,3))
        self.kernel = np.ones((3,3),np.uint8)
        self.gate_corners_flat = np.float32([[[94, 64],[204, 64],[204, 174], [94, 174]]])
        self.camera_matrix = np.array([[160.000, 0.000000, 160.000], [0.000000, 160.000, 120.000], [0.000000, 0.000000, 1.000000]])
        self.gate_center_pixel_flat = np.array([149,119,1]).T
        self.no_gate_count = 0
        self.measurement_count = 0
        self.close_count = 1
        self.wrong_gate_count = 0
        #self.lower_green = np.array([0, 210, 0])
        #self.upper_green = np.array([200, 255, 200])
        self.lower_green = np.array([0, 150, 0])
        self.upper_green = np.array([255, 255, 110])
        self.dist_coeff = np.zeros((1,5))
        self.waypoint_gate_1 = np.array([0.0, 0.0, +1.0, 1.0]).T.reshape(4,1)
        self.waypoint_gate_2 = np.array([0.0, 0.0, 0.0, 1.0]).T.reshape(4,1)
        self.waypoint_gate_3 = np.array([0.0, 0.0, -2.0, 1.0]).T.reshape(4,1)
        self.gate_points_3D = 1.5*np.array([[0,0,0],[-1.0,1.0,0],[1.0,1.0,0],[1.0,-1.0,0],[-1.0,-1.0,0]])
        self.viz_traj = viz_traj
        self.viz_traj_color_rgba = viz_traj_color_rgba
        self.drone_name = drone_name
        self.viz_image_cv2 = viz_image_cv2

    # Find area of gate
    def find_gate_area(self, x_coords ,y_coords):
        area_numerator = 0
        for i in range(4):
            if i == 3:
                area_numerator += (x_coords[i]*y_coords[0]- y_coords[i]*x_coords[0])
            else:
                area_numerator += (x_coords[i]*y_coords[i+1] - y_coords[i]*x_coords[i+1])
        gate_area = abs(area_numerator/2.0)
        return gate_area

    # Find four gate corners with largest area given more than four points
    def find_four_gate_corners_largest_area(self, gate_corners):
        gate_corners = gate_corners.reshape(len(gate_corners),2)
        gate_corner_combinations = list(combinations(gate_corners,4))
        largest_area = 0.0
        for comb in gate_corner_combinations:
            comb_array = np.empty((0,2))
            for i in comb:
                i = np.reshape(i,(1,2))
                comb_array = np.append(comb_array,i,axis=0)
            comb_array = order_points(comb_array)
            gate_area = self.find_gate_area(comb_array[:,0], comb_array[:,1])
            if gate_area > largest_area:
                largest_area = copy.deepcopy(gate_area)
                gate_corners_largest_area = copy.deepcopy(comb_array)
        gate_corners_largest_area = gate_corners_largest_area.astype(int).reshape(4,1,2)
        return gate_corners_largest_area

    # Find the highest aspect ratio of the gate
    # if aspect ratio is too high, a gate side may have been detected instead of a full gate
    def find_aspect_ratio(self, gate_corners):
        aspect_ratio_mtx = np.empty((1,4))
        aspect_ratio_mtx[0,0] = abs(np.float32(gate_corners[1][0][1] - gate_corners[2][0][1])/np.float32(gate_corners[0][0][0] - gate_corners[1][0][0]+self.eps))
        aspect_ratio_mtx[0,1] = abs(np.float32(gate_corners[0][0][1] - gate_corners[3][0][1])/np.float32(gate_corners[2][0][0] - gate_corners[3][0][0]+self.eps))
        aspect_ratio_mtx[0,2] = abs(np.float32(gate_corners[0][0][1] - gate_corners[3][0][1])/np.float32(gate_corners[0][0][0] - gate_corners[1][0][0]+self.eps))
        aspect_ratio_mtx[0,3] = abs(np.float32(gate_corners[1][0][1] - gate_corners[2][0][1])/np.float32(gate_corners[2][0][0] - gate_corners[3][0][0]+self.eps))
        large_aspect_ratio = 1.0
        for i in range(4):
            if aspect_ratio_mtx[0,i] < 1.0:
                aspect_ratio_mtx[0,i] = 1/aspect_ratio_mtx[0,i]
            if aspect_ratio_mtx[0,i] > self.aspect_ratio_max:
                large_aspect_ratio = aspect_ratio_mtx[0,i]
        return large_aspect_ratio

    def get_rotation_matrix_quad_frame_to_global_frame(self,state):
        q0 = state.orientation.w_val
        q1 = state.orientation.x_val
        q2 = state.orientation.y_val
        q3 = state.orientation.z_val
        rot_matrix_quad2global = np.array([[1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3-q0*q2)],
                                        [2*(q1*q2 + q0*q3), 1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)],
                                        [2*(q1*q3-q0*q2), 2*(q1*q0+q2*q3), 1-2*(q1**2+q2**2)]])
        return rot_matrix_quad2global

    def gate_frame_waypoint_to_global_waypoint(self,state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,waypoint_gate):
        waypoint_rel = np.dot(rot_translation_matrix_gate2quad, waypoint_gate)
        waypoint_rel = np.dot(rot_matrix_quad2global, np.array([waypoint_rel[2], waypoint_rel[0], -waypoint_rel[1]]))
        waypoint_glob = np.reshape(np.array([state.position.x_val + waypoint_rel[0], state.position.y_val + waypoint_rel[1], state.position.z_val - waypoint_rel[2]]),(1,3))
        return waypoint_glob

    def get_average_waypoint(self,measurement_estimates_mtx,waypoint_glob):
        measurement_estimates_mtx = np.append(measurement_estimates_mtx, waypoint_glob,axis=0)
        waypoint_ave = np.reshape(np.mean(measurement_estimates_mtx,axis=0),(1,3))
        return waypoint_ave

    def run(self):
        # Move through first gate
        self.get_ground_truth_gate_poses()
        # if self.plot_transform:
        #     self.airsim_client.plot_transform([self.gate_poses_ground_truth[0]], vehicle_name=self.drone_name)
        # self.airsim_client.moveOnSplineAsync([self.gate_poses_ground_truth[0].position], vel_max = 2.0, acc_max = 5.0, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
        self.airsim_client.moveOnSplineAsync([self.gate_poses_ground_truth[0].position], vel_max=2.0, acc_max=5.0, add_position_constraint=True, add_velocity_constraint=False,
            add_acceleration_constraint=False, viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, vehicle_name=self.drone_name)

        time.sleep(2)

        while self.airsim_client.isApiControlEnabled(vehicle_name=self.drone_name):
            # Take image
            response = self.airsim_client.simGetImages([airsim.ImageRequest("fpv_cam", airsim.ImageType.Scene, False, False)])
            img1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8) # get numpy array
            image_rgb = img1d.reshape(response[0].height, response[0].width, 3)

            # get rotation matrix from quad frame to global frame
            state = self.airsim_client.simGetVehiclePose()
            rot_matrix_quad2global = self.get_rotation_matrix_quad_frame_to_global_frame(state)

            # Find corners of the gate
            mask = cv2.inRange(image_rgb,self.lower_green,self.upper_green)
            #print(mask)
            dilated_gate = cv2.dilate(mask,self.kernel, iterations=8)
            eroded_gate = cv2.erode(dilated_gate,self.kernel, iterations=8)
            __, gate_contours, hierarchy = cv2.findContours(dilated_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # gate_contours, hierarchy = cv2.findContours(dilated_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            print("got image")
            cv2.imshow("mask", mask)
            cv2.imshow("dilated_gate", dilated_gate)
            cv2.imshow("eroded_gate", eroded_gate)
            cv2.waitKey(1)

            largest_area = 0.0
            gate_corners = None
            gate_corners_best = np.zeros((1,3))
            for contour in gate_contours:
                if cv2.contourArea(contour) > largest_area:
                    gate_contour_epsilon = 0.01 * cv2.arcLength(contour, True)
                    gate_contour_approx = cv2.approxPolyDP(contour,gate_contour_epsilon,True)
                    gate_corners = cv2.convexHull(gate_contour_approx)
                    if len(gate_corners) > 4: # check if there are more than four points given, find four corners that produce largest area
                        gate_corners = self.find_four_gate_corners_largest_area(gate_corners)
                    if len(gate_corners) == 4:
                        aspect_ratio = self.find_aspect_ratio(gate_corners)
                        if aspect_ratio <= self.aspect_ratio_max:
                            largest_area = cv2.contourArea(contour)
                            gate_corners_best = gate_corners

            # MOVE
            if largest_area < 800 or len(gate_corners_best)!=4:
                self.no_gate_count += 1
                print("Gate NOT detected")
                if self.no_gate_count > 50: # If no gate has been detected for over 50 attempts, rotate to look for gate.
                    self.airsim_client.moveByYawRateAsync(yaw_rate=-15.0, duration=0.05, vehicle_name=self.drone_name).join() # rotate counter clockwise to look for next gate

            else:
                self.no_gate_count = 0
                gate_corners_best = order_points(gate_corners_best.reshape(4,2))
                gate_corners_plot = gate_corners_best
                gate_corners_best = np.float32(gate_corners_best).reshape(-1,1,2)

                # find homography matrix between gate corner points in baseline image and corner points in image taken
                homography_matrix, mask = cv2.findHomography(gate_corners_best, self.gate_corners_flat, cv2.RANSAC, 5.0)

                # find middle of gate pixel coordinates
                gate_center_pixel = np.dot(np.linalg.inv(homography_matrix), self.gate_center_pixel_flat)
                gate_center_pixel = (int(gate_center_pixel[0]/(gate_center_pixel[2]+self.eps)), int(gate_center_pixel[1]/(gate_center_pixel[2]+self.eps)))

                # coordinates of gate corners in image (pixel coordinates)
                gate_points_2D = np.float32(np.concatenate((np.array(gate_center_pixel).reshape(-1,1,2),gate_corners_best),axis=0))

                # find rotation and translation from gate frame to quad frame
                __, rvec, tvec = cv2.solvePnP(self.gate_points_3D,gate_points_2D,self.camera_matrix,self.dist_coeff)
                rvec_full, __ = cv2.Rodrigues(rvec)
                rot_translation_matrix_gate2quad = np.concatenate((rvec_full, tvec), axis=1)
                rot_translation_matrix_gate2quad = np.concatenate((rot_translation_matrix_gate2quad, np.array([0,0,0,1]).reshape(1,4)), axis=0)

                # gate frame waypoint to global frame waypoint
                waypoint_glob_1 = self.gate_frame_waypoint_to_global_waypoint(state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,self.waypoint_gate_1)
                waypoint_glob_2 = self.gate_frame_waypoint_to_global_waypoint(state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,self.waypoint_gate_2)
                waypoint_glob_3 = self.gate_frame_waypoint_to_global_waypoint(state,rot_translation_matrix_gate2quad,rot_matrix_quad2global,self.waypoint_gate_3)

                # if quad is too close to next waypoint, move through gate without taking measurements
                quad_to_next_gate_dist = abs(np.linalg.norm(np.array([state.position.x_val,state.position.y_val,state.position.z_val]) - self.waypoint_ave_2))
                if quad_to_next_gate_dist < 3.0 and self.close_count == 0:
                    print("Too close to gate")
                    #if self.plot_transform:
                    #    self.airsim_client.plot_transform([airsim.Pose(waypoint_3, airsim.Quaternionr())], vehicle_name=self.drone_name)
                    #self.airsim_client.moveOnSplineAsync([waypoint_3], vel_max = 2.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
                    # self.airsim_client.moveOnSplineAsync([waypoint_3], vel_max=2.0, acc_max=2.0, add_position_constraint=True, add_velocity_constraint=False,
                    #     add_acceleration_constraint=False, viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, vehicle_name=self.drone_name)

                    time.sleep(3)
                    self.measurement_count = 0
                    self.wrong_gate_count = 0
                    self.close_count = 1
                    self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                else:
                    if self.close_count == 1: # right after passing through gate, reset the average
                        self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                        self.measurement_count = 0
                    self.close_count = 0
                    measurement_diff = abs(np.linalg.norm(waypoint_glob_2-self.waypoint_ave_2))
                    quad_to_measurement_dist = abs(np.linalg.norm(waypoint_glob_2-np.array([state.position.x_val,state.position.y_val,state.position.z_val])))
                    if measurement_diff > 0.5: # if new measurement is very far from previous measurements, wrong gate is measured
                        print("wrong gate", self.wrong_gate_count)
                        if self.wrong_gate_count > 10: # if wrong gate is measured over 10 times, reset the average
                            self.measurement_count = 0
                            self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                        self.wrong_gate_count += 1
                    elif quad_to_measurement_dist > 20: # if new measurement is very far from quad, wrong gate is measured
                        print("wrong gate", self.wrong_gate_count)
                        print(quad_to_measurement_dist)
                        if self.wrong_gate_count > 10: # if wrong gate is measured over 10 times, reset the average
                            self.measurement_count = 0
                            #self.airsim_client.moveByYawRateAsync(yaw_rate=-15.0, duration=0.05, vehicle_name=self.drone_name).join() # rotate counter clockwise to look for next gate
                            #self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                    else:
                        cv2.circle(image_rgb, (int(gate_center_pixel[0]), int(gate_center_pixel[1])), 10, (255, 0, 0), -1)
                        cv2.circle(image_rgb, (int(gate_corners_plot[0][0]), int(gate_corners_plot[0][1])), 10, (255, 100, 0), -1)
                        cv2.circle(image_rgb, (int(gate_corners_plot[1][0]), int(gate_corners_plot[1][1])), 10, (255, 0, 100), -1)
                        cv2.circle(image_rgb, (int(gate_corners_plot[2][0]), int(gate_corners_plot[2][1])), 10, (255, 200, 0), -1)
                        cv2.circle(image_rgb, (int(gate_corners_plot[3][0]), int(gate_corners_plot[3][1])), 10, (255, 0, 200), -1)
                        cv2.imshow("image_rgb", image_rgb)
                        cv2.waitKey(1)
                        self.wrong_gate_count = 0
                        self.measurement_count += 1
                        print("Gate detected, Measurement Taken")
                        print(self.measurement_count)

                        # Calculate average of gate waypoints
                        if self.measurement_count == 1: # reset average
                            measurement_estimates_mtx_1 = np.empty((0,3))
                            measurement_estimates_mtx_2 = np.empty((0,3))
                            measurement_estimates_mtx_3 = np.empty((0,3))
                        waypoint_ave_1 = self.get_average_waypoint(measurement_estimates_mtx_1,waypoint_glob_1)
                        self.waypoint_ave_2 = self.get_average_waypoint(measurement_estimates_mtx_2,waypoint_glob_2)
                        waypoint_ave_3 = self.get_average_waypoint(measurement_estimates_mtx_3,waypoint_glob_3)

                        # waypoints
                        waypoint_1 = airsim.Vector3r(waypoint_ave_1[0,0],waypoint_ave_1[0,1], waypoint_ave_1[0,2])
                        waypoint_2 = airsim.Vector3r(self.waypoint_ave_2[0,0],self.waypoint_ave_2[0,1], self.waypoint_ave_2[0,2])
                        waypoint_3 = airsim.Vector3r(waypoint_ave_3[0,0],waypoint_ave_3[0,1], waypoint_ave_3[0,2])

                        #if self.plot_transform:
                        #    self.airsim_client.plot_transform([airsim.Pose(waypoint_2, airsim.Quaternionr())], vehicle_name=self.drone_name)
                        #self.airsim_client.moveOnSplineAsync([waypoint_2], vel_max = 2.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
                        self.airsim_client.moveOnSplineAsync([waypoint_1, waypoint_2, waypoint_3], vel_max=2.0, acc_max=2.0, add_position_constraint=True, add_velocity_constraint=False,
                            add_acceleration_constraint=False, viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, vehicle_name=self.drone_name)

                        #self.airsim_client.moveOnSplineAsync([waypoint_1,waypoint_2,waypoint_3], vel_max = 2.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
                        #time.sleep(0.5)
            #time.sleep(0.5)

def main(args):
    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    #baseline_racer = BaselineRacerPerception(drone_name="drone_1", plot_transform=args.plot_transform, viz_traj=args.viz_traj)
    baseline_racer = BaselineRacerPerception(drone_name="drone_1", viz_traj=args.viz_traj, viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0], viz_image_cv2=args.viz_image_cv2)
    baseline_racer.load_level(args.level_name)
    baseline_racer.initialize_drone()
    baseline_racer.takeoff_with_moveOnSpline()
    baseline_racer.run()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy"], default="Soccer_Field_Easy")
    #parser.add_argument('--plot_transform', dest='plot_transform', action='store_true', default=False)
    parser.add_argument('--viz_traj', dest='viz_traj', action='store_true', default=False)
    parser.add_argument('--enable_viz_image_cv2', dest='viz_image_cv2', action='store_true', default=False)
    args = parser.parse_args()
    main(args)
