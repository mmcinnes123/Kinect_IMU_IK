# Run IMU IK with Kinect's body coordinate frames

import opensim as osim
import os
from helpers import *


""" SETTINGS """


# Define some file paths
parent_dir = os.getcwd()
results_dir = os.path.join(parent_dir, 'Results')
data_dir = os.path.join(parent_dir, 'Data')
IMU_IK_settings_template_file = os.path.join(parent_dir, 'Settings Files', 'IMU_IK_Settings.xml')
APDM_converter_settings_file = os.path.join(parent_dir, 'Settings Files', 'APDMDataConverter_Settings.xml')
APDM_template_csv = os.path.join(parent_dir, 'APDM_template_4S.csv')

# Calibration settings
template_model_file = 'das3_scaled_and_placed_middle.osim'
calibrated_model_file_name = 'Calibrated_das3_scaled_and_placed_middle.osim'


# IK Settings
oris_sto = 'x'  # Input orientations file
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
trim_bool = False
start_time = None
end_time = None
IK_output_file_name = os.path.join(results_dir, 'IK_Results.mot')
visualize_tracking = False



""" MAIN """

