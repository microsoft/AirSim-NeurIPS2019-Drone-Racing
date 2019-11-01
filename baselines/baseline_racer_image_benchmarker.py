from argparse import ArgumentParser
import airsimneurips as airsim
import time
import utils
import threading
import numpy as np
import cv2
from baseline_racer import BaselineRacer

class BaselineRacerImageBenchmarker(BaselineRacer):
    def __init__(self, 
            img_benchmark_type = 'simGetImage', 
            drone_name = "drone_1", 
            viz_traj = False, 
            viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0], 
            viz_image_cv2 = False):
        super().__init__(drone_name=drone_name, viz_traj=viz_traj, viz_image_cv2=viz_image_cv2)

        self.image_benchmark_num_images = 0
        self.image_benchmark_total_time = 0.0
        self.image_callback_thread = None
        if img_benchmark_type == "simGetImage":
            self.image_callback_thread = threading.Thread(target=self.repeat_timer_img, args=(self.image_callback_benchmark_simGetImage, 0.05))
        if img_benchmark_type == "simGetImages":
            self.image_callback_thread = threading.Thread(target=self.repeat_timer_img, args=(self.image_callback_benchmark_simGetImages, 0.05))
        self.is_image_thread_active = False

    def start_img_benchmark_thread(self):
        if not self.is_image_thread_active:
            self.is_image_thread_active = True
            self.image_callback_thread.start()
            print("Started img image_callback thread")

    def stop_img_benchmark_thread(self):
        if self.is_image_thread_active:
            self.is_image_thread_active = False
            self.image_callback_thread.join()
            print("Stopped image callback thread.")

    def repeat_timer_img(self, task, period):
        while self.is_image_thread_active:
            task()
            time.sleep(period)

    def print_benchmark_results(self):
        avg_fps = 1.0 / ((self.image_benchmark_total_time) / float(self.image_benchmark_num_images))
        print(self.level_name + ": {} avg_fps for {} num of images".format(avg_fps, self.image_benchmark_num_images))

    def image_callback_benchmark_simGetImage(self):
        self.image_benchmark_num_images += 1
        iter_start_time = time.time()
        response = self.airsim_client_images.simGetImage("fpv_cam", airsim.ImageType.Scene)
        img_rgb = cv2.imdecode(airsim.string_to_uint8_array(response), cv2.IMREAD_UNCHANGED)
        self.image_benchmark_total_time += time.time() - iter_start_time
        avg_fps = 1.0 / ((self.image_benchmark_total_time) / float(self.image_benchmark_num_images))
        print(self.level_name + ": {} avg_fps for {} num of images".format(avg_fps, self.image_benchmark_num_images))
        # uncomment following lines to viz image
        # if self.viz_image_cv2:
            # cv2.imshow("img_rgb", img_rgb_1d_new)
            # cv2.waitKey(1)

    def image_callback_benchmark_simGetImages(self):
        self.image_benchmark_num_images += 1
        iter_start_time = time.time()
        request = [airsim.ImageRequest("fpv_cam", airsim.ImageType.Scene, False, False)]
        response = self.airsim_client_images.simGetImages(request)
        img_rgb_1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8) 
        img_rgb = img_rgb_1d.reshape(response[0].height, response[0].width, 3)
        self.image_benchmark_total_time += time.time() - iter_start_time
        avg_fps = 1.0 / ((self.image_benchmark_total_time) / float(self.image_benchmark_num_images))
        print(self.level_name + ": {} avg_fps for {} num of images".format(avg_fps, self.image_benchmark_num_images))
        # uncomment following lines to viz image
        # if self.viz_image_cv2:
            # cv2.imshow("img_rgb", img_rgb_1d_new)
            # cv2.waitKey(1)

def main(args):
    # ensure you have generated the neurips planning settings file by running python generate_settings_file.py
    baseline_racer = BaselineRacerImageBenchmarker(img_benchmark_type=args.img_benchmark_type, \
        drone_name="drone_1", \
        viz_traj=args.viz_traj, \
        viz_traj_color_rgba=[1.0, 1.0, 0.0, 1.0], \
        viz_image_cv2=args.viz_image_cv2)
 
    baseline_racer.load_level(args.level_name)
    if args.level_name == "Qualifier_Tier_1":
        args.race_tier = 1
    if args.level_name == "Qualifier_Tier_2":
        args.race_tier = 2
    if args.level_name == "Qualifier_Tier_3":
        args.race_tier = 3

    baseline_racer.start_race(args.race_tier)
    baseline_racer.initialize_drone()
    baseline_racer.takeoff_with_moveOnSpline()
    baseline_racer.get_ground_truth_gate_poses()
    baseline_racer.start_img_benchmark_thread()
    baseline_racer.fly_through_all_gates_at_once_with_moveOnSpline().join()
    baseline_racer.stop_img_benchmark_thread()
    baseline_racer.print_benchmark_results()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--level_name', type=str, choices=["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard", 
        "Qualifier_Tier_1", "Qualifier_Tier_2", "Qualifier_Tier_3"], default="ZhangJiaJie_Medium")
    parser.add_argument('--enable_viz_traj', dest='viz_traj', action='store_true', default=False)
    parser.add_argument('--img_benchmark_type', type=str, choices=["simGetImage", "simGetImages"], default="simGetImages")
    parser.add_argument('--enable_viz_image_cv2', dest='viz_image_cv2', action='store_true', default=False)
    parser.add_argument('--race_tier', type=int, choices=[1,2,3], default=1)

    args = parser.parse_args()
    main(args)