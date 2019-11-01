import airsimneurips as airsim
import json
import numpy as np
import os

def to_airsim_vector(np_arr):
    assert np.size(np_arr) == 3
    return airsim.Vector3r(np.float(np_arr[0]), np.float(np_arr[1]), np.float(np_arr[2]))

def to_airsim_vectors(np_arr):
    return [to_airsim_vector(np_arr[i, :]) for i in range(np.size(np_arr, 0))]

# these clases are only meant to be settings generator.
# for everything else, there's airsimneurips.Pose()
class Position():
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x = x
        self.y = y
        self.z = z

class Rotation():
    def __init__(self, yaw = 0.0, pitch = 0.0, roll = 0.0):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

class Pose():
    def __init__(self, position, rotation):
        self.position = position
        self.rotation = rotation

class AirSimSettingsCreator(object):
    def __init__(self, sim_mode = "Multirotor"):
        self.sim_mode = sim_mode
        self.settings_dict = {}

    def add_minimal(self):
        self.settings_dict["SeeDocsAt"] = "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md"
        self.settings_dict["SettingsVersion"] = 1.2
        self.settings_dict["SimMode"] = self.sim_mode
        self.settings_dict["ClockSpeed"] = 1

    # can be used for camera pose or vehicle pose by passing in the right settings_key
    def set_pose(self, setting_key, pose):
        setting_key["X"] = pose.position.x
        setting_key["Y"] = pose.position.y
        setting_key["Z"] = pose.position.z
        setting_key["Pitch"] = pose.rotation.pitch
        setting_key["Roll"] = pose.rotation.roll
        setting_key["Yaw"] = pose.rotation.yaw

    def add_multirotor(self, vehicle_name, pose):
        assert(self.settings_dict["SimMode"] == "Multirotor")
        if "Vehicles" not in self.settings_dict.keys():
            self.settings_dict['Vehicles'] = {}

        self.settings_dict['Vehicles'][vehicle_name] = {}
        self.settings_dict['Vehicles'][vehicle_name]["VehicleType"] = "SimpleFlight"
        self.set_pose(self.settings_dict['Vehicles'][vehicle_name], pose)

    def add_camera(self, vehicle_name, camera_name, relative_pose, image_type, image_width, image_height, fov_horizontal_degrees):
        # fetch vehicle setting dict
        vehicle_setting = self.settings_dict['Vehicles'][vehicle_name]
        # initialize vehicle's camera setting dict to empty 
        vehicle_setting['Cameras'] = {}
        vehicle_setting['Cameras'][camera_name] = {}
        camera_setting = vehicle_setting['Cameras'][camera_name]
        self.set_pose(camera_setting, relative_pose)
        capture_setting = {}
        capture_setting['Width'] = image_width
        capture_setting['Height'] = image_height
        capture_setting['ImageType'] = image_type
        capture_setting['FOV_Degrees'] = fov_horizontal_degrees
        camera_setting['CaptureSettings'] = [capture_setting]

    # default linux: /home/$USER/Documents/AirSim/settings.json
    # default windows: C:\\Users\\%USERNAME%\\Documents\\AirSim\\settings.json
    def write_airsim_settings_file(self, base_filename="settings.json"):
        user_dir = os.path.expanduser("~")
        airsim_settings_dir = os.path.join(user_dir, "Documents", "AirSim")
        if not os.path.exists(airsim_settings_dir):
            os.makedirs(airsim_settings_dir)
        airsim_settings_abs_file_path = os.path.join(airsim_settings_dir, base_filename)
        with open(airsim_settings_abs_file_path, "w") as f:
            json.dump(self.settings_dict, f, indent=2, sort_keys=True)

    # usage: AirSimSettingsCreator().write_airsim_neurips_baseline_settings_file()
    def write_airsim_neurips_baseline_settings_file(self):
        instance = self.__class__()
        instance.add_minimal()
        instance.add_multirotor(vehicle_name = "drone_1", pose = Pose(Position(), Rotation()))
        instance.add_camera(vehicle_name = "drone_1", camera_name = 'fpv_cam', relative_pose=Pose(Position(0.25, 0.0, 0.0), Rotation()), 
                image_type = 0, image_width = 320, image_height = 240, fov_horizontal_degrees = 90)
        instance.add_multirotor(vehicle_name = "drone_2", pose = Pose(Position(), Rotation()))
        instance.write_airsim_settings_file()