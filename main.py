# Script to run IMU IK with Kinect's body coordinate frames

import os
from helpers import *


""" SETTINGS """

# Input csv file
Kinect_body_quat_csv_file = 'sflex-k_quats.csv'

# Define some folders
parent_dir = os.getcwd()
results_dir = os.path.join(parent_dir, 'Results')
data_dir = os.path.join(parent_dir, 'Data')
settings_templates_dir = os.path.join(parent_dir, 'Settings and Templates')

# Define some file paths
APDM_converter_settings_file = os.path.join(settings_templates_dir, 'APDMDataConverter_Settings.xml')
APDM_template_csv = os.path.join(settings_templates_dir, 'APDM_template_4S.csv')
Kinect_body_quat_csv_file_path = os.path.join(data_dir, Kinect_body_quat_csv_file)

# IK Settings
IMU_IK_settings_template_file = os.path.join(parent_dir, 'Settings and Templates', 'IMU_IK_Settings.xml')
calibrated_model_file_name = 'Calibrated_das3_scaled_and_placed_middle.osim'
orientations_file = os.path.join(data_dir, 'Kinect_Body_Quats_all.sto')   # This will be created when you run write_movements_and_calibration_stos()
trim_bool = False   # Specify whether to trim the data before running IK
start_time = None   # Enter start time for trim function in seconds
end_time = None     # Enter end time for trim function in seconds
IK_output_file_name = os.path.join(results_dir, 'IK_Results.mot')   # The name of the IK results .mot file


""" MAIN """

# Convert the .csv quaternion data into an .sto file so that OpenSim can read it
convert_csv_ori_data_to_sto(Kinect_body_quat_csv_file_path, data_dir, APDM_template_csv, APDM_converter_settings_file)


# Run IMU Inverse Kinematics
run_osim_IMU_IK(IMU_IK_settings_template_file, calibrated_model_file_name, orientations_file,
                results_dir, trim_bool, start_time, end_time, IK_output_file_name)