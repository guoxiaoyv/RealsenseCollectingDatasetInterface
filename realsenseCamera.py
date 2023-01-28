import cv2
import numpy as np
import pyrealsense2 as rs


# Проверка на тип сенсора
# rs.sensor.is_depth_sensor(self.stereo_module)
# Серийный номер устройства
# str(self.device.get_info(rs.camera_info.serial_number))


class RealSenseCamera:

    def __init__(self, stereo_profile_id=0,
                 color_profile_id=0,
                 motion_profile_id=0):

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)

        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.stereo_module = self.device.sensors[0]
        self.rgb_camera = self.device.sensors[1]
        self.motion_module = self.device.sensors[2]

        self.__profile = {
            "stereo": {
                "fps": rs.video_stream_profile(self.stereo_module.profiles[stereo_profile_id]).fps(),
                "format": rs.video_stream_profile(self.stereo_module.profiles[stereo_profile_id]).format(),
                "stream_type": rs.video_stream_profile(self.stereo_module.profiles[stereo_profile_id]).stream_type(),
                "resolution": (rs.video_stream_profile(self.stereo_module.profiles[stereo_profile_id]).width(),
                               rs.video_stream_profile(self.stereo_module.profiles[stereo_profile_id]).height())
            },

            "color": {
                "fps": rs.video_stream_profile(self.rgb_camera.profiles[color_profile_id]).fps(),
                "format": rs.video_stream_profile(self.rgb_camera.profiles[color_profile_id]).format(),
                "stream_type": rs.video_stream_profile(self.rgb_camera.profiles[color_profile_id]).stream_type(),
                "resolution": (rs.video_stream_profile(self.rgb_camera.profiles[color_profile_id]).width(),
                               rs.video_stream_profile(self.rgb_camera.profiles[color_profile_id]).height())
            },

            "infrared": {

            }
        }

        # depth
        self.config.enable_stream(self.__profile["stereo"]["stream_type"],
                                  self.__profile["stereo"]["resolution"][0],
                                  self.__profile["stereo"]["resolution"][1],
                                  self.__profile["stereo"]["format"],
                                  self.__profile["stereo"]["fps"])

        # rgb
        self.config.enable_stream(self.__profile["color"]["stream_type"],
                                  self.__profile["color"]["resolution"][0],
                                  self.__profile["color"]["resolution"][1],
                                  self.__profile["color"]["format"],
                                  self.__profile["color"]["fps"])

        # motion
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)

        # infrared
        self.config.enable_stream(rs.video_stream_profile(self.stereo_module.profiles[0]).stream_type(),
                                  rs.video_stream_profile(self.stereo_module.profiles[0]).width(),
                                  rs.video_stream_profile(self.stereo_module.profiles[0]).height(),
                                  rs.video_stream_profile(self.stereo_module.profiles[0]).format(),
                                  rs.video_stream_profile(self.stereo_module.profiles[0]).fps())

        self.__depth_image = None
        self.__motion_accel = None
        self.__motion_gyro = None

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        infrared_frame = frames.get_infrared_frame()
        self.__motion_accel = frames[3].as_motion_frame().get_motion_data()
        self.__motion_gyro = frames[4].as_motion_frame().get_motion_data()

        for frame in frames:
            print(frame)

        self.__depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        infrared_image = np.asanyarray(infrared_frame.get_data())

        return self.__depth_image, color_image, infrared_image

    def get_accel(self):
        return np.array([self.__motion_accel.x, self.__motion_accel.y, self.__motion_accel.z])

    def get_gyro(self):
        return np.array([self.__motion_gyro.x, self.__motion_gyro.y, self.__motion_gyro.z])

    def get_distance(self, x, y):
        return self.__depth_image[x, y]

    def start(self):
        self.pipeline.start(self.config)

    def stop(self):
        self.pipeline.stop()
