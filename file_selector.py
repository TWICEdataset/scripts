import pandas as pd
import os
import pathlib


def file_selector(scenario, weather, scenario_type, radar_mode, test_run, camera_type):
    with open('TWICE_path.txt', 'r') as f:
        path_twice = f.read()
    if scenario == os.path.join("CBLA","with_car") or scenario == os.path.join("CBLA","withot_car"):
        scenario_df = "CBLA"
    else:
        scenario_df = scenario

    df = pd.read_csv(os.path.join(path_twice, 'TWICE', 'scripts', 'TWICE_df.csv'))

    df_selected = df[(df['Scenario'] == scenario_df) & (df['Type'] == scenario_type) & (df['Radar_mode'] == radar_mode)]
    weather_list = df_selected['Weather'].tolist()
    if weather not in weather_list:
        print('This weather was not recorded under the selected conditions. Please select one of the following weather:', weather_list)

    df_selected = df[(df['Weather'] == weather) & (df['Scenario'] == scenario_df) & (df['Type'] == scenario_type) & (df['Radar_mode'] == radar_mode)]
    test_run_list = df_selected['Test_run'].tolist()

    if test_run not in test_run_list:
        print('This test_run does not exist. Please select one of the following numbers:', test_run_list)

    # dynamic or static ego
    if scenario == 'CCRb' or scenario == os.path.join("CBLA","with_car") or scenario == os.path.join('CBLA','without_car') or scenario == 'CCRs' or scenario == 'truck_perpendicular':
        scenario_path = os.path.join(path_twice, 'TWICE', 'Scenarios', 'dynamic_ego', scenario, scenario_type, weather, radar_mode, f'test_run_{test_run}')
    else:
        scenario_path = os.path.join(path_twice, 'TWICE', 'Scenarios', 'static_ego', scenario, scenario_type, weather, radar_mode, f'test_run_{test_run}')

    # real or synthetic data
    if scenario_type == 'real': 
        camera_path = os.path.join(scenario_path, 'Camera', 'camera_sv_350_300.osi')
        lidar_path = os.path.join(scenario_path, 'Lidar', 'lidar_sd_350_300.osi')

    if scenario_type == 'synthetic':
        if camera_type == "DDI":
            camera_path = os.path.join(scenario_path, 'Camera_DDI', 'camera_sv_350_300.osi')
        if camera_type == "OTA":
            camera_path = os.path.join(scenario_path, 'Camera_OTA', 'camera_sv_350_300.osi')
        lidar_path = 'Does not exist'

    obj_path = os.path.join(scenario_path, 'IMU_obj', 'obj_sv_350_300.osi')
    ego_path = os.path.join(scenario_path, 'IMU_ego', 'ego_sv_350_300.osi')
    radar_path = os.path.join(scenario_path, 'Radar', 'radar_sd_350_300.osi')

    if scenario =="CCRs":
        obj_path = os.path.join(scenario_path, 'Static_car', 'obj_sv_350_300.osi')

    #warning lack of data and interpolation
    for root, subfolders, filenames in os.walk(scenario_path):
        for filename in filenames:
            filepath = os.path.join(root , filename)
            if filepath.endswith(".txt"):
                with open(filepath, 'r') as f:
                    test_run_warning = f.read()
                print(test_run_warning)

    return(camera_path,radar_path,ego_path,obj_path,lidar_path)

# weather = "rain"
# scenario = "bike"
# scenario_type = "real"
# radar_mode = "cluster"

# file_selector(scenario,weather,scenario_type,radar_mode)

def file_selector_path(path_ego):
    path_name = pathlib.PurePath(path_ego)
    list_elements = list(path_name.parts)
    scenario_path = os.path.dirname(os.path.dirname(path_ego))
    # real or synthetic data
    if 'real' in list_elements: 
        camera_path = os.path.join(scenario_path, 'Camera', 'camera_sv_350_300.osi')
        lidar_path = os.path.join(scenario_path, 'Lidar', 'lidar_sd_350_300.osi')

    if "synthetic" in list_elements:
        camera_path = [
            os.path.join(scenario_path, 'Camera_DDI', 'camera_sv_350_300.osi'),
            os.path.join(scenario_path, 'Camera_OTA', 'camera_sv_350_300.osi')
        ]
        lidar_path = 'Does not exist'

    obj_path = os.path.join(scenario_path, 'IMU_obj', 'obj_sv_350_300.osi')
    ego_path = os.path.join(scenario_path, 'IMU_ego', 'ego_sv_350_300.osi')
    radar_path = os.path.join(scenario_path, 'Radar', 'radar_sd_350_300.osi')

    #warning lack of data and interpolation
    for root, subfolders, filenames in os.walk(scenario_path):
        for filename in filenames:
            filepath = os.path.join(root , filename)
            if filepath.endswith(".txt"):
                with open(filepath, 'r') as f:
                    test_run_warning = f.read()
                print(test_run_warning)

    return(camera_path,radar_path,ego_path,obj_path,lidar_path)