# TWICE_dataset
This repository contains the scripts to open files of the TWICE dataset. 
This repository contains the updated scripts to open files of the TWICE dataset: https://github.com/TWICEdataset/scripts
There is a python notebook named open_osi_files.ipynb that contain an example of how to manipulate the data.
You can use these scripts to project cuboids onto frames and save them in video files.
You can also save a single camera frame with or without the projected cuboids. 
Additionally, a script is available for converting LiDAR data from .osi to .ply point cloud format. 
For radar data, it is possible to create a video file that includes plot data from the radar, as well as the ground truth for both the ego and the object of interest.
# Requirements
- This scripts must be inside the folder TWICE, like this:

TWICE <br>
│ <br>
├── scripts <br>
├── Scenarios <br>
├── open-simulation-interface <br>


- Follow instructions of OSI Installation [available at here](https://www.asam.net/static_downloads/ASAM_OSI_reference-documentation_v3.5.0/index.html)
1. pip install --upgrade google-api-python-client

To ensure that this script works correctly, yoou will need to change the path in the TWICE_path.txt to reflect the location of the TWICE folder on your system. (path where the folder TWICE is stored in your system)
