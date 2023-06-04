from threading import Thread
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import time
import os


class CameraStream:
    def __init__(self, resolution=(640, 480), framerate=30):

        self.datasetName = "ZoomOut"  # Change directory name for dataset output
        self.unitsInSeconds = True
        self.color_image = np.zeros((resolution[0], resolution[1]))
        self.depth_image = np.zeros((resolution[0], resolution[1]))
        self.camera_stopped = False
        self.resolution = resolution
        self.framerate = framerate

        # Do Not Modify
        self.imu_pipe = rs.pipeline()
        self.imu_config = rs.config()
        self.imu_config.enable_stream(
            rs.stream.gyro, rs.format.motion_xyz32f, 200)
        self.imu_config.enable_stream(
            rs.stream.accel, rs.format.motion_xyz32f, 250)
        self.acc = []
        self.gyro = []

        self.previous_timestamp = 0
        self.begin_time = time.time()
        self.vid_pipe = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(
            rs.stream.depth, resolution[0], resolution[1], rs.format.z16, framerate)
        self.config.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, framerate)

        # Check if directory exists, create if not
        if '{}'.format(self.datasetName) not in os.listdir():
            os.mkdir('{}'.format(self.datasetName))
            os.mkdir('{}/rgb'.format(self.datasetName))
            os.mkdir('{}/depth'.format(self.datasetName))

        self.fileIMU = open("{}/imu.txt".format(self.datasetName), "a")
        self.fileIMU.write(
            "# IMU data\n# file:{}\n# timestamp gx gy gz ax ay az\n".format(self.datasetName))

        self.fileDepth = open("{}/depth.txt".format(self.datasetName), "a")
        self.fileRGB = open("{}/rgb.txt".format(self.datasetName), "a")

        self.fileRGB.write(
            "# color images\n# file: {}\n# timestamp filename\n".format(self.datasetName))
        self.fileDepth.write(
            "# depth images\n# file: {}\n# timestamp filename\n".format(self.datasetName))

    def start_camera(self):
        # start the thread to read frames from the video stream
        profile = self.vid_pipe.start(self.config)
        intrinsics = profile.get_stream(
            rs.stream.depth).as_video_stream_profile().get_intrinsics()
        print(intrinsics)
        depth_sensor = profile.get_device().query_sensors()[1]
        depth_sensor.set_option(rs.option.enable_auto_exposure, True)
        for i in range(0, 6):  # 20 works here
            self.vid_pipe.wait_for_frames()

        Thread(target=self.update_cam).start()

    def start_imu(self):
        # start the thread to read frames from the video stream
        self.imu_pipe.start(self.imu_config)
        for i in range(0, 1):  # 200 works here
            self.imu_pipe.wait_for_frames()

        Thread(target=self.update_imu).start()

    def update_cam(self):

        print("Starting Camera Stream")
        while True:
            # Wait for a coherent pair of frames: depth and color
            vid_frames = self.vid_pipe.wait_for_frames()
            depth_frame = vid_frames.get_depth_frame()
            color_frame = vid_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            self.depth_image = np.asanyarray(depth_frame.get_data()) * 5
            self.color_image = np.asanyarray(color_frame.get_data())

            if self.unitsInSeconds:
                # Converts milliseconds to seconds
                stamp = "{:.6f}".format(depth_frame.get_timestamp() / 1e3)[4:]
            else:
                # Converts milliseconds to nanoseconds
                stamp = "{:.6f}".format(depth_frame.get_timestamp() * 1e6)[6:]

            fileName = "{}.png".format(stamp)
            cv2.imwrite(
                '{}/rgb/{}'.format(self.datasetName, fileName), self.color_image)
            cv2.imwrite(
                '{}/depth/{}'.format(self.datasetName, fileName), self.depth_image)

            self.fileDepth.write(stamp + " depth/" + fileName + "\n")
            self.fileRGB.write(stamp + " rgb/" + fileName + "\n")

            # print()

    def update_imu(self):
        print("Starting IMU Stream")
        while True:
            # Wait for a coherent pair of frames: acceleration and gyroscope data
            mot_frames = self.imu_pipe.wait_for_frames()
            self.acc = mot_frames[0].as_motion_frame().get_motion_data()
            self.gyro = mot_frames[1].as_motion_frame().get_motion_data()

            tempTime1 = mot_frames[0].get_timestamp() / 1e3  # *1e6
            tempTime2 = mot_frames[1].get_timestamp() / 1e3  # *1e6
            sentTimeStamp = tempTime1
            sentTimeStamp = tempTime1 if (
                tempTime1 > tempTime2) else tempTime2
            if (sentTimeStamp == self.previous_timestamp):
                continue

            self.previous_timestamp = sentTimeStamp

            if self.unitsInSeconds:
                # Converts milliseconds into seconds
                stamp = "{:.6f}".format(sentTimeStamp)[4:]
            else:
                # Converts milliseconds to nanoseconds
                stamp = "{:.6f}".format(
                    mot_frames[1].get_timestamp() * 1e6)[6:]

            self.fileIMU.write(
                stamp + " {0} {1} {2} {3} {4} {5}\n".format(self.gyro.x, self.gyro.y, self.gyro.z, self.acc.x, self.acc.y, self.acc.z))


if __name__ == '__main__':

    sensor = CameraStream()
    sensor.start_imu()
    sensor.start_camera()

    try:
        while True:
            # Shows RGB version of what sensor currently sees
            cv2.imshow("image", sensor.color_image)
            # print("acc, gyro", vs.acc, vs.gyro)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        sensor.fileIMU.close()
        sensor.fileRGB.close()
        sensor.fileDepth.close()
        sensor.imu_pipe.stop()
        sensor.vid_pipe.stop()
