# Run IMU IK with Kinect's body coordinate frames

import opensim as osim
import os
from helpers import *


""" SETTINGS """

# Define some file paths
parent_dir = os.getcwd()
results_dir = os.path.join(parent_dir, 'Results')
data_dir = os.path.join(parent_dir, 'Data')
IMU_IK_settings_template_file = os.path.join(parent_dir, 'Settings and Templates', 'IMU_IK_Settings.xml')
APDM_converter_settings_file = os.path.join(parent_dir, 'Settings and Templates', 'APDMDataConverter_Settings.xml')
APDM_template_csv = os.path.join(parent_dir, 'Settings and Templates', 'APDM_template_4S.csv')

# Define the input data file names
Kinect_body_quat_csv_file = os.path.join(data_dir, 'sflex-k_quats.csv')
Kinect_sto_file = os.path.join(data_dir, 'Kinect_Body_Quats_all.sto')   # This will be created when you run write_movements_and_calibration_stos()

# Calibration settings
calibrated_model_file_name = 'Calibrated_das3_scaled_and_placed_middle.osim'


# IK Settings
oris_sto = 'x'  # Input orientations file
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)      # Rotate the Kinext orientation data to match OpenSim's y-up global frame
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
trim_bool = False   # Specify whether to trim the data before running IK
start_time = None   # Enter start time for trim function in seconds
end_time = None     # Enter end time for trim function in seconds
IK_output_file_name = os.path.join(results_dir, 'IK_Results.mot')
visualize_tracking = False

""" MAIN """


# Convert the .csv quaternion data into an .sto file so that OpenSim can read it
convert_csv_ori_data_to_sto(Kinect_body_quat_csv_file, data_dir, APDM_template_csv, APDM_converter_settings_file)



# Run IMU Inverse Kinematics
orientations_data = Kinect_sto_file
run_osim_IMU_IK(IMU_IK_settings_template_file, calibrated_model_file_name, orientations_data,
               sensor_to_opensim_rotations, results_dir, trim_bool,
               start_time, end_time, IK_output_file_name, visualize_tracking)