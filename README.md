
***Recording A Dataset (TUM Format) With D435i on Windows Laptop***


**System Requirements:**

Windows 10

Python Installed (At least 3.7 or higher))

Vscode (for convenience but not necessary)

**Getting Started (Before attempting to run anything):**

**Required Libraries:**

Make sure you have the pyrealsense2, cv2, and numpy libraries installed on the device you are connecting the camera to for streaming and collecting data. Also make sure that the dependencies for these are also installed, the terminal will notify you of which ones might be missing. 

These can be installed with the following commands, and note that the version numbers need not be as specific (most recent versions will work):

pip install pyrealsense2
(Version=2.53.1.4623)

pip install opencv-python
(Version=4.5.5.64)

pip install numpy
(Version=1.22.3)
**File Structure: **


Your file structure (in the src directory) before beginning recording should look something like this:

src/
    - ThreadStream.py
**Setting parameters**
See comments in script file for where to modify parameters such as framerate, resolution, etc.

**Running stream and recording:**

First, CD into the src folder. 
Then, when you are ready to record, run the script using ‘python ThreadStream.py’ or ‘python3 ThreadStream.py’, depending on which version you have as the executable. 

You should receive two messages acknowledging that the RGB/Depth stream has started, along with the IMU. 

If you receive a “No frames received within 5000 seconds error,” try unplugging and replugging the D435I connection cable. 

The dataset should start being collected, and when you want to end the recording, simply press Ctrl+C (At least twice in succession) to kill the process. 

A new directory should have been generated with the name of your dataset as entered in the script, and look something like this:

&lt;datasetName>/ -

      - rgb/

      - depth/

      - imu.txt

      - rgb.txt

      - depth.txt

**Format Changes Needed for ORBSLAM3:**

For ORBSLAM3, you will need a file with a list of the rgb timestamps separately, and for this you can use the get_timestamps.py file included in this repo. Place it in the dataset directory, and run the script in that directory on your terminal. This should create an ‘rgb_timestamps.txt’ file. 

**Troubleshooting with the D435i:**

Due to potential hardware issues, first thing to try is simply to disconnect and reconnect the cable connecting the sensor and the computer used. 

Some combinations of configuration settings will simply not work. Refer to the D435i manual for limitations. These include attempting to run at 60FPS with a resolution greater than 640 by 480.  These may result in “frames not received” errors or other ones. RGB will not work more than 30FPS when streaming concurrently with IMU, and it is still unsure whether depth will. 
