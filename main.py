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

# Define the input data file names
Kinect_body_quat_csv_file = os.path.join(data_dir, 'sflex-k_quats.csv')

Kinect_sto_file = os.path.join(data_dir, 'Kinect_Body_Quats_all.sto')   # This will be created when you run write_movements_and_calibration_stos()

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


# Convert the .csv quaternion data into an .sto file so that OpenSim can read it
write_movements_and_calibration_stos(Kinect_body_quat_csv_file, data_dir, APDM_template_csv,
                                     APDM_converter_settings_file)


### I manually added some IMUs into the model xml file


# Calibrate the model
# Use 'manual' calibration, not OpenSim's built-in IMU calibration, which is based on an intial pose
# The 'manual' calibration essentially defines the input Kinect CFs as being perfectly aligned with the
# bodies we're trying to track

thorax_virtual_IMU = get_IMU_cal_MANUAL('Thorax')
humerus_virtual_IMU = get_IMU_cal_MANUAL('Humerus')
radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file, results_dir)


# Run IMU Inverse Kinematics
orientations_data = Kinect_sto_file
run_osim_IMU_IK(IMU_IK_settings_template_file, calibrated_model_file_name, orientations_data,
               sensor_to_opensim_rotations, results_dir, trim_bool,
               start_time, end_time, IK_output_file_name, visualize_tracking)