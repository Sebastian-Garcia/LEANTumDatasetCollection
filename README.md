
** Recording A Dataset (TUM Format) With D435i on Windows Laptop **


**System Requirements:**

Windows 10

Python Installed (3.7 or higher preferred)

Vscode (for convenience but not necessary)

**Getting Started (Before attempting to run anything):**

**Required Libraries:**

Make sure you have the pyrealsense2, cv2, and numpy libraries installed on the device you are connecting the camera to for streaming and collecting data. Also make sure that the dependencies for these are also installed, the terminal will notify you of which ones might be missing. 

These can be installed with the following commands:

pip install pyrealsense2

pip install opencv-python

pip install numpy

**File Structure: **

Place the ThreadStream.py file in the directory (folder) where you would like to save your data to. Create an rgb subfolder, and depth subfolder.

Your file structure before beginning recording should look something like this:

datasetName/ -

      - rgb/

      - depth/

      - ThreadStream.py

**Setting Parameters:**

**	**

**Running stream and recording:**

When you are ready to record, simply run the script using ‘python ThreadStream.py’ or ‘python3 ThreadStream.py’, depending on which version you have as the executable. 

You should receive two messages acknowledging that the RGB/Depth stream has started, along with the IMU. 

If you receive a “No frames received within 5000 seconds error,” try unplugging and replugging the D435I connection cable. 

The dataset should start being collected, and when you want to end the recording, simply press Ctrl+C (At least twice in succession) to kill the process. 

Your file structure should now look something like this:

&lt;datasetName>/ -

      - rgb/

      - depth/

      - ThreadStream.py

      - imu.txt

      - rgb.txt

      - depth.txt

**Post-processing:**

For ORBSLAM3, you will need a file with a list of the rgb timestamps separately, and for this you can use the get_timestamps.py file included in this repo. Place it in the dataset directory, and run the script in that directory on your terminal. This should create an ‘rgb_timestamps.txt’ file. 

**Troubleshooting with the D435i:**

Some combinations of configuration settings will simply not work. These may result in “frames not received” errors or other ones. RGB will not work more than 30FPS when streaming concurrently with IMU, and it is still unsure whether depth will. 
