#!/bin/bash

#------------------------------------
# Monocular-Inertial Examples
SEQ_NAME=Desk2
CONFIG_PATH=./Monocular-Inertial/RealSense_D435i.yaml
#RGB_PATH=/data_raid5/zihsing/datasets/roomWithTinyFurniture/rgb_ns
RGB_PATH=../../$SEQ_NAME/rgb
#TIMESTAMPS_PATH=/data_raid5/zihsing/datasets/roomWithTinyFurniture/rgb_timestamps.txt
TIMESTAMPS_PATH=../../$SEQ_NAME/rgb_timestamps.txt
#IMU_PATH=/data_raid5/zihsing/datasets/roomWithTinyFurniture/IMU_post.txt
IMU_PATH=../../$SEQ_NAME/imu.txt
OUT_TAG=$SEQ_NAME

echo "Launching roomWithTinyFurniture with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt $CONFIG_PATH $RGB_PATH $TIMESTAMPS_PATH $IMU_PATH $OUT_TAG