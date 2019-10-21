import airsimneurips
import threading
import time

class ReproduceResetRaceCondition():
    def __init__(self, drone_name = "drone_1"):
        self.airsim_client = airsimneurips.MultirotorClient()
        self.airsim_client_2 = airsimneurips.MultirotorClient()
        self.airsim_client_3 = airsimneurips.MultirotorClient()
        self.drone_name = drone_name
        self.is_thread_active = False
        self.thread_reset = threading.Thread(target=self.repeat_timer, args=(self.reset, 0.05))
        self.thread_reset_race = threading.Thread(target=self.repeat_timer, args=(self.reset_race, 0.03))
        self.thread_reset_and_reset_race = threading.Thread(target=self.repeat_timer, args=(self.reset_and_reset_race, 0.09))
        self.is_thread_active = False

    def repeat_timer(self, callback, period):
        while self.is_thread_active:
            callback()
            time.sleep(period)

    def load_level(self, level_name, sleep_sec = 2.0):
        self.level_name = level_name
        self.airsim_client.simLoadLevel(self.level_name)
        self.airsim_client.confirmConnection() # failsafe
        time.sleep(sleep_sec) # let the environment load completely

    def reset(self):
        print(time.time(), 'called reset')
        self.airsim_client.reset()

    def reset_race(self):
        print(time.time(), 'called simResetRace')
        self.airsim_client_2.simResetRace()

    def reset_and_reset_race(self):
        print(time.time(), 'called reset, followed by simResetRace')
        self.airsim_client_3.reset()
        self.airsim_client_3.simResetRace()

    def start_race(self, tier):
        print(time.time(), 'called start race')
        self.airsim_client.simStartRace(tier)

    def initialize_drone(self):
        self.airsim_client.enableApiControl(vehicle_name=self.drone_name)
        self.airsim_client.arm(vehicle_name=self.drone_name)

        # set default values for trajectory tracker gains 
        traj_tracker_gains = airsimneurips.TrajectoryTrackerGains(kp_cross_track = 5.0, kd_cross_track = 0.0, 
                                                            kp_vel_cross_track = 3.0, kd_vel_cross_track = 0.0, 
                                                            kp_along_track = 0.4, kd_along_track = 0.0, 
                                                            kp_vel_along_track = 0.04, kd_vel_along_track = 0.0, 
                                                            kp_z_track = 2.0, kd_z_track = 0.0, 
                                                            kp_vel_z = 0.4, kd_vel_z = 0.0, 
                                                            kp_yaw = 3.0, kd_yaw = 0.1)

        self.airsim_client.setTrajectoryTrackerGains(traj_tracker_gains, vehicle_name=self.drone_name)
        time.sleep(0.2)

    def start_threads(self):
        if not self.is_thread_active:
            self.is_thread_active = True
            self.thread_reset.start()
            self.thread_reset_race.start()
            self.thread_reset_and_reset_race.start()
            print("Started threads")

    def stop_threads(self):
        if self.is_thread_active:
            self.is_thread_active = False
            self.thread_reset.join()
            self.thread_reset_race.join()
            self.thread_reset_and_reset_race.join()
            print("Stopped threads.")

if __name__ == "__main__":
    reproducer = ReproduceResetRaceCondition('drone_1')
    reproducer.load_level('Qualifier_Tier_1')
    reproducer.initialize_drone()
    reproducer.start_race(3)
    time.sleep(5)
    reproducer.start_threads()
    time.sleep(3600)
    reproducer.stop_threads()
