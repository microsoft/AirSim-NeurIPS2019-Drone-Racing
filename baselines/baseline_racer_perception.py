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
    def __init__(self, drone_name = "drone_1", plot_transform=False, viz_traj=False):
        BaselineRacer.__init__(self, drone_name, plot_transform, viz_traj)
        self.eps = 0.01
        self.waypoint_ave_2 = np.zeros((1,3))
        self.kernel = np.ones((3,3),np.uint8)
        self.dst_pts = np.float32([[[94, 64],[204, 64],[204, 174], [94, 174]]])
        self.camera_matrix = np.array([[160.000, 0.000000, 160.000], [0.000000, 160.000, 120.000], [0.000000, 0.000000, 1.000000]])
        self.gate_center_pixel_flat = np.array([149,119,1]).T
        self.no_gate_count = 0
        self.measurement_count = 0
        self.close_count = 1
        self.wrong_gate_count = 0
        self.lower_green = np.array([0, 210, 0])
        self.upper_green = np.array([200, 255, 200])
        self.dist_coeff = np.zeros((1,5))
        self.waypoint_gate_1 = np.array([0.0,0.0,+1.0,1.0]).T.reshape(4,1)
        self.waypoint_gate_2 = np.array([0.0,0.0,0.0,1.0]).T.reshape(4,1)
        self.waypoint_gate_3 = np.array([0,0.0,-2.0,1.0]).T.reshape(4,1)
        self.objpoints = 1.5*np.array([[0,0,0],[-1.0,1.0,0],[1.0,1.0,0],[1.0,-1.0,0],[-1.0,-1.0,0]])

    # Find area of polygon
    def find_poly_area(self, x ,y):
        return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

    # Find four gate corners given more than four points
    # use the four points that produce the largest area
    def find_gate_corners(self, gate_corners):
        gate_corners = gate_corners.reshape(len(gate_corners),2)
        all_combs = list(combinations(gate_corners,4))
        comb_count_out = 0
        largest_area = 0.0
        for comb in all_combs:
            comb_count_out += 1
            comb_count = 0
            for i in comb:
                comb_count += 1
                if comb_count == 1:
                    comb_array = i
                else:
                    comb_array = np.vstack((comb_array,i))
            poly_area = self.find_poly_area(comb_array[:,0], comb_array[:,1])
            if poly_area > largest_area:
                largest_area = copy.deepcopy(poly_area)
                comb_array_largest = copy.deepcopy(comb_array)
        comb_array_largest = comb_array_largest.astype(int).reshape(4,1,2)
        return comb_array_largest

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
            if aspect_ratio_mtx[0,i] > 1.4:
                large_aspect_ratio = aspect_ratio_mtx[0,i]
        return large_aspect_ratio

    def run(self):
        # Move through first gate
        self.get_ground_truth_gate_poses()
        #self.airsim_client.plot_transform([self.gate_poses_ground_truth[0]], vehicle_name=self.drone_name)
        self.airsim_client.moveOnSplineAsync([self.gate_poses_ground_truth[0].position], vel_max = 2.0, acc_max = 5.0, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
        time.sleep(2)
        print(self.gate_poses_ground_truth[1])
        while self.airsim_client.isApiControlEnabled(vehicle_name=self.drone_name):
            # Take image
            response = self.airsim_client.simGetImages([airsim.ImageRequest("fpv_cam", airsim.ImageType.Scene, False, False)])
            img1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8) # get numpy array
            image_rgb = img1d.reshape(response[0].height, response[0].width, 3)

            # Get quad state when the image was taken
            state = self.airsim_client.simGetVehiclePose()
            q0 = state.orientation.w_val
            q1 = state.orientation.x_val
            q2 = state.orientation.y_val
            q3 = state.orientation.z_val

            # rotation matrix between quad body frame and global frame
            rt_mtx_quad2global = np.array([[1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3-q0*q2)],
                                            [2*(q1*q2 + q0*q3), 1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)],
                                            [2*(q1*q3-q0*q2), 2*(q1*q0+q2*q3), 1-2*(q1**2+q2**2)]])

            # Find corners of the gate
            mask = cv2.inRange(image_rgb,self.lower_green,self.upper_green)
            dilated_gate = cv2.dilate(mask,self.kernel, iterations=8)
            eroded_gate = cv2.erode(dilated_gate,self.kernel, iterations=8)
            __, gate_contours, hierarchy = cv2.findContours(dilated_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # gate_contours, hierarchy = cv2.findContours(dilated_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # __, gate_contours, hierarchy = cv2.findContours(dilated_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            cv2.imshow("mask", mask)
            cv2.imshow("dilated_gate", dilated_gate)
            cv2.imshow("eroded_gate", eroded_gate)
            cv2.waitKey(1)
            largest_area = 0.0
            gate_corners = None
            gate_corners_best = np.zeros((1,3))
            for c in gate_contours:
                if cv2.contourArea(c) > largest_area:
                    self.epsilon_comp = 0.01 * cv2.arcLength(c, True)
                    gate_contour_approx = cv2.approxPolyDP(c,self.epsilon_comp,True)
                    gate_corners = cv2.convexHull(gate_contour_approx)
                    # check if there are more than four points given; if there are, find four points that produce largest area polygon
                    if len(gate_corners) > 4:
                        gate_corners = self.find_gate_corners(gate_corners)
                    if len(gate_corners) == 4:
                        aspect_ratio = self.find_aspect_ratio(gate_corners)
                        if aspect_ratio <= 1.4:
                            largest_area = cv2.contourArea(c)
                            gate_contour_comp = c
                            gate_corners_best = gate_corners

            # MOVE
            if largest_area < 500 or len(gate_corners_best)!=4:
                self.no_gate_count += 1
                #self.wrong_gate_count = 0
                print("Gate NOT detected")
                # If no gate has been detected for over 25 attempts, rotate to look for gate
                if self.no_gate_count > 25:
                    self.airsim_client.moveByYawRateAsync(yaw_rate=-15.0, duration=2, vehicle_name=self.drone_name) # rotate counter clockwise to look for next gate

            else:
                self.no_gate_count = 0
                src_pts = order_points(gate_corners_best.reshape(4,2))
                gate_corners_plot = src_pts
                src_pts = np.float32(src_pts).reshape(-1,1,2)

                # find homography matrix between gate corner points in baseline image and corner points in image taken
                homography_matrix, mask = cv2.findHomography(src_pts, self.dst_pts, cv2.RANSAC, 5.0)

                # find middle of gate pixel coordinates
                gate_center_pixel = np.dot(np.linalg.inv(homography_matrix), self.gate_center_pixel_flat)
                gate_center_pixel = (int(gate_center_pixel[0]/(gate_center_pixel[2]+self.eps)), int(gate_center_pixel[1]/(gate_center_pixel[2]+self.eps)))

                # coordinates of gate corners in image (pixel coordinates)
                imgpoints = np.float32(np.concatenate((np.array(gate_center_pixel).reshape(-1,1,2),src_pts),axis=0))

                # find rotation/translation matrix between image to gate centered frame
                ret, rvecs, tvecs = cv2.solvePnP(self.objpoints,imgpoints,self.camera_matrix,self.dist_coeff)
                rvec_full, jac1 = cv2.Rodrigues(rvecs)
                rt_mtx_gate2quad = np.concatenate((rvec_full, tvecs), axis=1)
                rt_mtx_gate2quad = np.concatenate((rt_mtx_gate2quad, np.array([0,0,0,1]).reshape(1,4)), axis=0)

                # relative location of gate from quad to global location of gate
                waypoint_rel_1 = np.dot(rt_mtx_gate2quad, self.waypoint_gate_1)
                waypoint_rel_1 = np.dot(rt_mtx_quad2global, np.array([waypoint_rel_1[2], waypoint_rel_1[0], -waypoint_rel_1[1]]))

                waypoint_rel_2 = np.dot(rt_mtx_gate2quad, self.waypoint_gate_2)
                waypoint_rel_2 = np.dot(rt_mtx_quad2global, np.array([waypoint_rel_2[2], waypoint_rel_2[0], -waypoint_rel_2[1]]))

                waypoint_rel_3 = np.dot(rt_mtx_gate2quad,self.waypoint_gate_3)
                waypoint_rel_3 = np.dot(rt_mtx_quad2global, np.array([waypoint_rel_3[2], waypoint_rel_3[0], -waypoint_rel_3[1]]))

                waypoint_glob_1 = np.reshape(np.array([state.position.x_val + waypoint_rel_1[0], state.position.y_val + waypoint_rel_1[1], state.position.z_val - waypoint_rel_1[2]]),(1,3))
                waypoint_glob_2 = np.reshape(np.array([state.position.x_val + waypoint_rel_2[0], state.position.y_val + waypoint_rel_2[1], state.position.z_val - waypoint_rel_2[2]]),(1,3))
                waypoint_glob_3 = np.reshape(np.array([state.position.x_val + waypoint_rel_3[0], state.position.y_val + waypoint_rel_3[1], state.position.z_val - waypoint_rel_3[2]]),(1,3))

                # if quad is too close to next waypoint, move through gate
                self.quad_dist_error = abs(np.linalg.norm(np.array([state.position.x_val,state.position.y_val,state.position.z_val]) - self.waypoint_ave_2))
                if self.quad_dist_error < 3.0 and self.close_count == 0:
                    print("too close to gate")
                    self.airsim_client.moveOnSplineAsync([waypoint_3], vel_max = 2.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
                    time.sleep(3)
                    self.measurement_count = 0
                    self.wrong_gate_count = 0
                    self.close_count += 1
                    self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                else:
                    if self.close_count == 1: # right after passing through gate, reinitialize
                        self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                        self.measurement_count = 0
                    self.close_count = 0
                    norm_dist_error = abs(np.linalg.norm(waypoint_glob_2-self.waypoint_ave_2))
                    quad_meas_error = abs(np.linalg.norm(waypoint_glob_2-np.array([state.position.x_val,state.position.y_val,state.position.z_val])))
                    # if new measurement is very far from previous measurement, wrong gate is measured
                    if norm_dist_error > 1.0:
                        print(norm_dist_error, quad_meas_error)
                        print("wrong gate", self.wrong_gate_count)
                        # if wrong gate is measured over 10 times, reinitialize next waypoint
                        if self.wrong_gate_count > 10:
                            self.measurement_count = 0
                            self.waypoint_ave_2 = copy.deepcopy(waypoint_glob_2)
                        self.wrong_gate_count += 1
                    # if new measurement is very far from quad, wrong gate is measured
                    elif quad_meas_error > 20:
                        print(norm_dist_error, quad_meas_error)
                        print("wrong gate", self.wrong_gate_count)
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
                        # calculate moving average of gate waypoints
                        if self.measurement_count == 1:
                            measurement_estimates_mtx_1 = np.reshape(waypoint_glob_1,(1,3))
                            measurement_estimates_mtx_2 = np.reshape(waypoint_glob_2,(1,3))
                            measurement_estimates_mtx_3 = np.reshape(waypoint_glob_3,(1,3))
                            waypoint_ave_1 = np.reshape(waypoint_glob_1,(1,3))
                            self.waypoint_ave_2 = np.reshape(waypoint_glob_2,(1,3))
                            waypoint_ave_3 = np.reshape(waypoint_glob_3,(1,3))
                        else:
                            measurement_estimates_mtx_1 = np.append(measurement_estimates_mtx_1, waypoint_glob_1,axis=0)
                            measurement_estimates_mtx_2 = np.append(measurement_estimates_mtx_2, waypoint_glob_2,axis=0)
                            measurement_estimates_mtx_3 = np.append(measurement_estimates_mtx_3, waypoint_glob_3,axis=0)
                            waypoint_ave_1 = np.reshape(np.mean(measurement_estimates_mtx_1,axis=0),(1,3))
                            self.waypoint_ave_2 = np.reshape(np.mean(measurement_estimates_mtx_2,axis=0),(1,3))
                            waypoint_ave_3 = np.reshape(np.mean(measurement_estimates_mtx_3,axis=0),(1,3))

                        # three waypoints
                        waypoint_1 = airsim.Vector3r(waypoint_ave_1[0,0],waypoint_ave_1[0,1], waypoint_ave_1[0,2])
                        waypoint_2 = airsim.Vector3r(self.waypoint_ave_2[0,0],self.waypoint_ave_2[0,1], self.waypoint_ave_2[0,2])
                        waypoint_3 = airsim.Vector3r(waypoint_ave_3[0,0],waypoint_ave_3[0,1], waypoint_ave_3[0,2])

                        #self.airsim_client.moveOnSplineAsync([waypoint_1,waypoint_2,waypoint_3], vel_max = 2.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, viz_traj=self.viz_traj, vehicle_name=self.drone_name)
                        self.airsim_client.moveOnSplineAsync([waypoint_2], vel_max = 2.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=True, viz_traj=self.viz_traj, vehicle_name=self.drone_name)

def main(args):
    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    baseline_racer = BaselineRacerPerception(drone_name="drone_1", plot_transform=args.plot_transform, viz_traj=args.viz_traj)
    baseline_racer.load_level(args.level_name)
    baseline_racer.initialize_drone()
    baseline_racer.takeoff_with_moveOnSpline()
    baseline_racer.run()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy"], default="Soccer_Field_Easy")
    parser.add_argument('--plot_transform', dest='plot_transform', action='store_false', default=False)
    parser.add_argument('--viz_traj', dest='viz_traj', action='store_false', default=False)
    args = parser.parse_args()
    main(args)
