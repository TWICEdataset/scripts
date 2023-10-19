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
2. Open simulation interface (OSI) - [Installation instructions](https://www.asam.net/static_downloads/ASAM_OSI_reference-documentation_v3.5.0/index.html)
3. pip install --upgrade google-api-python-client

## TWICE home page
https://twice.eletrica.ufpr.br<br>
https://twicedataset.github.io/site

## Dataset

Subset with examples used in tutorial.ipynb (439 MB): [TWICEsubset.zip](https://twice.eletrica.ufpr.br/TWICEsubset.zip)<br>
Full dataset (90 GB): [TWICE.zip](https://twice.eletrica.ufpr.br/TWICE.zip)<br>


## Reference
L. Novicki Neto et al., “TWICE Dataset: Digital Twin of Test Scenarios in a Controlled Environment.” arXiv, Oct. 05, 2023. doi: 10.48550/arXiv.2310.03895.