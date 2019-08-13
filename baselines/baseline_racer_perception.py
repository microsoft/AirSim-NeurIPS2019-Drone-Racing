from itertools import combinations
from imutils.perspective import order_points

import airsimneurips as airsim
import numpy as np
import time
import cv2
import copy
import math

# Find area of polygon
def find_poly_area(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

# Find four gate corners given more than four points
# use the four points that produce the largest area
def find_gate_corners(hull_comp):
    hull_comp_new = hull_comp.reshape(len(hull_comp),2)
    all_combs = list(combinations(hull_comp_new,4))
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
        poly_area = find_poly_area(comb_array[:,0],comb_array[:,1])
        if poly_area > largest_area:
            largest_area = copy.deepcopy(poly_area)
            comb_array_largest = copy.deepcopy(comb_array)
    return comb_array_largest

# Find the highest aspect ratio of the gate
# if aspect ratio is too high, a gate side may have been detected instead of a full gate
def find_aspect_ratio(hull_comp):
    aspect_ratio_mtx = np.empty((1,4))
    aspect_ratio_mtx[0,0] = abs(np.float32(hull_comp[1][0][1] - hull_comp[2][0][1])/np.float32(hull_comp[0][0][0] - hull_comp[1][0][0]+eps))
    aspect_ratio_mtx[0,1] = abs(np.float32(hull_comp[0][0][1] - hull_comp[3][0][1])/np.float32(hull_comp[2][0][0] - hull_comp[3][0][0]+eps))
    aspect_ratio_mtx[0,2] = abs(np.float32(hull_comp[0][0][1] - hull_comp[3][0][1])/np.float32(hull_comp[0][0][0] - hull_comp[1][0][0]+eps))
    aspect_ratio_mtx[0,3] = abs(np.float32(hull_comp[1][0][1] - hull_comp[2][0][1])/np.float32(hull_comp[2][0][0] - hull_comp[3][0][0]+eps))
    large_aspect_ratio = 1.0
    for i in range(4):
        if aspect_ratio_mtx[0,i] < 1.0:
            aspect_ratio_mtx[0,i] = 1/aspect_ratio_mtx[0,i]
        if aspect_ratio_mtx[0,i] > 1.2:
            large_aspect_ratio = aspect_ratio_mtx[0,i]
    return large_aspect_ratio

img_count = 1
no_gate_count = 0
eps = 0.01
waypoint_ave_2 = np.zeros((1,3))
last_good_waypoint = airsim.Vector3r(0,0,0)
kernel = np.ones((3,3),np.uint8)
#dst_pts = np.float32([[[240, 149],[399, 149],[399, 308], [240, 308]]])
dst_pts = np.float32([[[200, 132],[412, 132],[412, 346], [200, 346]]])

# connect to airsim
drone_name = "drone_1" # should match settings.json
client = airsim.MultirotorClient()
client.confirmConnection()
client.simLoadLevel('Soccer_Field_Easy')
time.sleep(2.0)
client.enableApiControl(vehicle_name=drone_name)
client.arm(vehicle_name=drone_name)
client.setTrajectoryTrackerGains(airsim.TrajectoryTrackerGains().to_list(), vehicle_name=drone_name)
client.moveOnSplineAsync([airsim.Vector3r(0, 0, -0.3)], vel_max=5.0, acc_max=3.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=False, viz_traj=True, vehicle_name=drone_name).join()

ctr = 0
while client.isApiControlEnabled(vehicle_name=drone_name) == True:
    img_count += 1

    # Move through first gate
    if img_count == 2:
        gate_names_sorted_bad = sorted(client.simListSceneObjects("Gate.*"))
        gate_indices_bad = [int(gate_name.split('_')[0][4:]) for gate_name in gate_names_sorted_bad]
        gate_indices_correct = sorted(range(len(gate_indices_bad)), key=lambda k:gate_indices_bad[k])
        gate_names = [gate_names_sorted_bad[gate_idx] for gate_idx in gate_indices_correct]
        gate_poses_rel = [client.simGetObjectPose(gate_name).position for gate_name in gate_names]
        waypoint_next = gate_poses_rel[0]
        # client.plot_tf([airsim.Pose(position_val=waypoint_next, orientation_val=airsim.Quaternionr())], duration=3.0, vehicle_name=drone_name)
        client.moveOnSplineAsync([waypoint_next], vel_max = 2.0, acc_max = 5.0, viz_traj=True, vehicle_name=drone_name).join()
        continue

    # Take image
    response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    img1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8) # get numpy array
    image_rgb = img1d.reshape(response[0].height, response[0].width, 3)

    # Get quad state when the image was taken
    state = client.simGetVehiclePose()
    q0 = state.orientation.w_val
    q1 = state.orientation.x_val
    q2 = state.orientation.y_val
    q3 = state.orientation.z_val

    # rotation/translation matrix between quad body frame and global frame
    rt_mtx_quad2global = np.array([[1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3-q0*q2)],
                                    [2*(q1*q2 + q0*q3), 1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)],
                                    [2*(q1*q3-q0*q2), 2*(q1*q0+q2*q3), 1-2*(q1**2+q2**2)]])

    # Process image
    # Find greenest pixels (pixels with gate)
    lower_green = np.array([0, 210, 0])
    upper_green = np.array([200, 255, 200])
    # Find corners of the gate
    mask = cv2.inRange(image_rgb,lower_green,upper_green)
    dilated_comp = cv2.dilate(mask,kernel, iterations=8)
    eroded_comp = cv2.erode(dilated_comp,kernel, iterations=8)
    # im_comp, contours_comp, hierarchy = cv2.findContours(dilated_comp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_comp, hierarchy = cv2.findContours(dilated_comp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # im_comp, contours_comp, hierarchy = cv2.findContours(dilated_comp, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cv2.imshow("mask", mask)
    cv2.imshow("dilated_comp", dilated_comp)
    cv2.imshow("eroded_comp", eroded_comp)
    #cv2.imshow("im_comp", im_comp)
    cv2.waitKey(1)
    largest_area = 0.0
    hull_comp = None
    hull_comp_best = np.zeros((1,3))
    for c in contours_comp:
        # print("area", cv2.contourArea(c))
        if cv2.contourArea(c) > largest_area:
            epsilon_comp = 0.01 * cv2.arcLength(c, True)
            approx_comp = cv2.approxPolyDP(c,epsilon_comp,True)
            hull_comp = cv2.convexHull(approx_comp)
            # check if there are more than four points given; if there are find four points that produce largest area polygon
            if len(hull_comp)>4:
                hull_comp = find_gate_corners(hull_comp)
                hull_comp = hull_comp.astype(int).reshape(4,1,2)
            if len(hull_comp)>=4:
                aspect_ratio = find_aspect_ratio(hull_comp)
                if aspect_ratio <= 1.2:
                    largest_area = cv2.contourArea(c)
                    gate_contour_comp = c
                    hull_comp_best = hull_comp

    # MOVE
    if largest_area < 3500 or len(hull_comp_best)!=4:
        no_gate_count += 1
        print("Gate NOT detected")
        # If no gate has been detected for over 25 attempts, move to the last good waypoint
        if no_gate_count > 25:
            client.moveOnSplineAsync([last_good_waypoint + airsim.Vector3r(np.random.uniform(low=-0.25,high=0.25,size=1)[0],
                                        np.random.uniform(low=-0.25,high=0.25,size=1)[0],
                                        np.random.uniform(low=-0.25,high=0.25,size=1)[0])],
                                    vel_max = 10.0, acc_max = 5.0, viz_traj=True, vehicle_name=drone_name).join() # move to last good waypoint
            no_gate_count = 0
        # Rotate counterclockwise until gate has been found
        else:
            client.moveByYawRateAsync(yaw_rate=-15.0, duration=0.5, vehicle_name=drone_name).join() # rotate counter clockwise to look for next gate
    else:
        # gate corner points
        src_pts = order_points(hull_comp_best.reshape(4,2))
        gate_corners_plot = src_pts
        src_pts = np.float32(src_pts).reshape(-1,1,2)

        # find homography matrix between gate corner points in baseline image and corner points in image taken
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        # find middle of gate pixel coordinates
        center_pixel_0 = np.array([306,239,1]).T
        center_pixel_comp = np.dot(np.linalg.inv(M), center_pixel_0)
        center_pixel_comp_norm = (int(center_pixel_comp[0]/(center_pixel_comp[2]+eps)), int(center_pixel_comp[1]/(center_pixel_comp[2]+eps)))

        print("center_pixel_comp", center_pixel_comp[0], center_pixel_comp[1], center_pixel_comp[2])
        print("center_pixel_comp_norm", center_pixel_comp_norm[0], center_pixel_comp_norm[1])
        cv2.circle(image_rgb, (int(center_pixel_comp_norm[0]), int(center_pixel_comp_norm[1])), 10, (255, 0, 0), -1)
        cv2.circle(image_rgb, (int(gate_corners_plot[0][0]), int(gate_corners_plot[0][1])), 10, (255, 100, 0), -1)
        cv2.circle(image_rgb, (int(gate_corners_plot[1][0]), int(gate_corners_plot[1][1])), 10, (255, 0, 100), -1)
        cv2.circle(image_rgb, (int(gate_corners_plot[2][0]), int(gate_corners_plot[2][1])), 10, (255, 200, 0), -1)
        cv2.circle(image_rgb, (int(gate_corners_plot[3][0]), int(gate_corners_plot[3][1])), 10, (255, 0, 200), -1)
        cv2.imshow("image_rgb", image_rgb)
        ctr+=1
        cv2.waitKey(1)

        # find 3D coordinate relative to quad
        # camera matrix
        mtx = np.array([[320.000, 0.000000, 320.000],
                        [0.000000, 320.000, 240.000],
                        [0.000000, 0.000000, 1.000000]])
        dist_coeff = np.zeros((1,5))

        # coordinates of three waypoints in gate centered frame
        waypoint_gate_1 = np.array([0.0,0.0,+1.0,1.0]).T.reshape(4,1)
        waypoint_gate_2 = np.array([0.0,0.0,0.0,1.0]).T.reshape(4,1)
        waypoint_gate_3 = np.array([0,0.0,-2.0,1.0]).T.reshape(4,1)

        # coordinates of gate corners in gate centered frame
        objpoints = 1.5*np.array([[0,0,0],[-1.0,1.0,0],[1.0,1.0,0],[1.0,-1.0,0],[-1.0,-1.0,0]])

        # coordinates of gate corners in image (pixel coordinates)
        imgpoints = np.concatenate((np.array(center_pixel_comp_norm).reshape(-1,1,2),src_pts),axis=0)
        imgpoints = np.float32(imgpoints)

        # find rotation/translation matrix between image to gate centered frame
        ret, rvecs, tvecs = cv2.solvePnP(objpoints,imgpoints,mtx,dist_coeff)
        rvec_full, jac1 = cv2.Rodrigues(rvecs)
        rt_mtx_gate2quad = np.concatenate((rvec_full, tvecs), axis=1)
        rt_mtx_gate2quad = np.concatenate((rt_mtx_gate2quad, np.array([0,0,0,1]).reshape(1,4)), axis=0)

        # relative location of gate from quad to global location of gate
        waypoint_rel_1 = np.dot(rt_mtx_gate2quad, waypoint_gate_1)
        waypoint_rel_1 = np.dot(rt_mtx_quad2global, np.array([waypoint_rel_1[2], waypoint_rel_1[0], -waypoint_rel_1[1]]))

        waypoint_rel_2 = np.dot(rt_mtx_gate2quad, waypoint_gate_2)
        waypoint_rel_2 = np.dot(rt_mtx_quad2global, np.array([waypoint_rel_2[2], waypoint_rel_2[0], -waypoint_rel_2[1]]))

        waypoint_rel_3 = np.dot(rt_mtx_gate2quad,waypoint_gate_3)
        waypoint_rel_3 = np.dot(rt_mtx_quad2global, np.array([waypoint_rel_3[2], waypoint_rel_3[0], -waypoint_rel_3[1]]))

        waypoint_glob_1 = np.reshape(np.array([state.position.x_val + waypoint_rel_1[0], state.position.y_val + waypoint_rel_1[1], state.position.z_val - waypoint_rel_1[2]]),(1,3))
        waypoint_glob_2 = np.reshape(np.array([state.position.x_val + waypoint_rel_2[0], state.position.y_val + waypoint_rel_2[1], state.position.z_val - waypoint_rel_2[2]]),(1,3))
        waypoint_glob_3 = np.reshape(np.array([state.position.x_val + waypoint_rel_3[0], state.position.y_val + waypoint_rel_3[1], state.position.z_val - waypoint_rel_3[2]]),(1,3))

        # check if your new measurement is far from your previous measurements
        norm_dist_error = abs(np.linalg.norm(waypoint_glob_2) - np.linalg.norm(waypoint_ave_2))
        if norm_dist_error > 0.5: #reset if your new measurement is too far from average waypoint
            measurement_count = 0

        # calculate moving average of gate waypoints
        measurement_count += 1
        print("Gate detected, Measurement Taken")
        print(measurement_count)
        if measurement_count == 1:
            measurement_estimates_mtx_1 = np.reshape(waypoint_glob_1,(1,3))
            measurement_estimates_mtx_2 = np.reshape(waypoint_glob_2,(1,3))
            measurement_estimates_mtx_3 = np.reshape(waypoint_glob_3,(1,3))
            waypoint_ave_1 = np.reshape(waypoint_glob_1,(1,3))
            waypoint_ave_2 = np.reshape(waypoint_glob_2,(1,3))
            waypoint_ave_3 = np.reshape(waypoint_glob_3,(1,3))
        else:
            measurement_estimates_mtx_1 = np.append(measurement_estimates_mtx_1, waypoint_glob_1,axis=0)
            measurement_estimates_mtx_2 = np.append(measurement_estimates_mtx_2, waypoint_glob_2,axis=0)
            measurement_estimates_mtx_3 = np.append(measurement_estimates_mtx_3, waypoint_glob_3,axis=0)
            waypoint_ave_1 = np.reshape(np.mean(measurement_estimates_mtx_1,axis=0),(1,3))
            waypoint_ave_2 = np.reshape(np.mean(measurement_estimates_mtx_2,axis=0),(1,3))
            waypoint_ave_3 = np.reshape(np.mean(measurement_estimates_mtx_3,axis=0),(1,3))

        # three waypoints
        waypoint_1 = airsim.Vector3r(waypoint_ave_1[0,0],waypoint_ave_1[0,1], waypoint_ave_1[0,2])
        waypoint_2 = airsim.Vector3r(waypoint_ave_2[0,0],waypoint_ave_2[0,1], waypoint_ave_2[0,2])
        waypoint_3 = airsim.Vector3r(waypoint_ave_3[0,0],waypoint_ave_3[0,1], waypoint_ave_3[0,2])

        no_gate_count = 0

        client.moveOnSplineAsync([waypoint_1,waypoint_2,waypoint_3], vel_max = 5.0, acc_max = 2.0, add_curr_odom_position_constraint=True, add_curr_odom_velocity_constraint=False, viz_traj=True, vehicle_name=drone_name).join()
        last_good_waypoint = copy.deepcopy(waypoint_3)
