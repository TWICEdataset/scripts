# TWICE_dataset
This repository contains the updated scripts to open files of the TWICE dataset: https://github.com/TWICEdataset/scripts
There is a python notebook tutorial.ipynb that contain examples of how to manipulate the data.
You can use these scripts to project cuboids onto frames and save them in video files.
You can also save a single camera frame with or without the projected cuboids. 
Additionally, a script is available for converting LiDAR data from .osi to .ply point cloud format. 
For radar data, it is possible to create a video file that includes plot data from the radar, as well as the ground truth for both the ego and the object of interest.

# Structure

TWICE <br>
│ <br>
├── scripts <br>
├── Scenarios <br>
├── open-simulation-interface <br>


# Requirements:
1. pip install --upgrade opencv-python tqdm open3d
2. OSI Installation - Follow instructions [available at here](https://www.asam.net/static_downloads/ASAM_OSI_reference-documentation_v3.5.0/index.html)
3. pip install --upgrade google-api-python-client

## TWICE home page
https://twicedataset.github.io/site

## Reference
L. Novicki Neto et al., “TWICE Dataset: Digital Twin of Test Scenarios in a Controlled Environment.” arXiv, Oct. 05, 2023. doi: 10.48550/arXiv.2310.03895.