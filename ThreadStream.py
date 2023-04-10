from threading import Thread
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import time
import os


class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):

        self.datasetName = "Desk1"

        self.color_image = np.zeros((resolution[0], resolution[1]))
        self.depth_image = self.color_image
        self.camera_stopped = False
        self.resolution = resolution
        self.framerate = framerate

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
        if 'dataset' not in os.listdir():
            os.mkdir('dataset')
            os.mkdir('dataset/rgb')
            os.mkdir('dataset/depth')

        self.fileIMU = open("dataset/imu.txt", "a")
        self.fileIMU.write(
            "# IMU data\n# file:{}\n# timestamp gx gy gz ax ay az\n".format(self.datasetName))

        self.fileDepth = open("dataset/depth.txt", "a")
        self.fileRGB = open("dataset/rgb.txt", "a")

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

            # stamp = "{:.6f}".format(depth_frame.get_timestamp() * 1e6)[6:]
            stamp = "{:.0f}".format(depth_frame.get_timestamp() * 1e6)[4:]
            fileName = "{}.png".format(stamp)
            cv2.imwrite(
                'dataset/rgb/{}'.format(fileName), self.color_image)
            cv2.imwrite(
                'dataset/depth/{}'.format(fileName), self.depth_image)

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

            tempTime1 = mot_frames[0].get_timestamp() * 1e6  # / 1e3  # *1e6
            tempTime2 = mot_frames[1].get_timestamp() * 1e6  # / 1e3  # *1e6
            sentTimeStamp = tempTime1
            sentTimeStamp = tempTime1 if (
                tempTime1 > tempTime2) else tempTime2
            if (sentTimeStamp == self.previous_timestamp):
                continue

            # print(1.0/(sentTimeStamp/1e9 - self.previous_timestamp/1e9))
            self.previous_timestamp = sentTimeStamp

            # stamp = "{:.6f}".format(mot_frames[1].get_timestamp() * 1e6)[6:]
            stamp = "{:.0f}".format(sentTimeStamp)[4:]
            self.fileIMU.write(
                stamp + " {0} {1} {2} {3} {4} {5}\n".format(self.gyro.x, self.gyro.y, self.gyro.z, self.acc.x, self.acc.y, self.acc.z))


if __name__ == '__main__':

    vs = VideoStream()
    vs.start_imu()
    vs.start_camera()

    try:
        while True:
            cv2.imshow("image", vs.color_image)
            # print("acc, gyro", vs.acc, vs.gyro)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        vs.fileIMU.close()
        vs.fileRGB.close()
        vs.fileDepth.close()
        vs.imu_pipe.stop()
        vs.vid_pipe.stop()
        print('Got here')
