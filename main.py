# Run IMU IK with Kinect's body coordinate frames

import opensim as osim
import os
from helpers import *


""" SETTINGS """
# Calibration settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
calibration_settings_template_file = "IMU_Calibration_Settings.xml"
template_model_file = 'das3_scaled_and_placed_middle.osim'

# Define some file paths
parent_dir = os.getcwd()
results_dir = os.path.join(parent_dir, 'Results')
data_dir = os.path.join(parent_dir, 'Data')

# Input orientations file
oris_sto = 'x'

""" MAIN """

### I manually added some IMUs into the model xml file


# Calibrate the model
# Use 'manual' calibration, not OpenSim's built-in IMU calibration, which is based on an intial pose
# The 'manual' calibration essentially defines the input Kinect CFs as being perfectly aligned with the
# bodies we're trying to track

thorax_virtual_IMU = get_IMU_cal_MANUAL('Thorax')
humerus_virtual_IMU = get_IMU_cal_MANUAL('Humerus')
radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file, results_dir)

