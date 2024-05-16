"""
This script defines the orientation of the virtual IMUs in the model.
 'Calibrating the model' is equivalent to defining the orientation of each virtual IMU which is rigidly
 attached to its corresponding model body.
 The 'virtual_IMU' rotation has been chosen, specific to each body, based on the orientation of the body CF,
 and the orientation of the equivalent body CF in kinect.
 """

from scipy.spatial.transform import Rotation as R
from helpers import apply_cal_to_model

# Specify the template model file. NOTE: I manually added some virtual IMUs into the model xml file first
template_model_file = 'das3_scaled_and_placed_middle.osim'



# Define a function which specifies a rotational offset between the body CF and the virtual IMU CF
def get_IMU_cal(which_body):

    if which_body == "Thorax":
        virtual_IMU = R.from_euler('XYZ', [-90, -90, 0], degrees=True)

    elif which_body == "Humerus":
        virtual_IMU = R.from_euler('XYZ', [90, 90, 0], degrees=True)

    elif which_body == "Radius":
        virtual_IMU = R.from_euler('XYZ', [0, 0, 90], degrees=True)

    else:
        print("Which_body input wasn't 'Thorax', 'Humerus', or 'Radius'")
        virtual_IMU = None

    return virtual_IMU



# Get the rotation offset of the virtual IMU for each body
thorax_virtual_IMU = get_IMU_cal('Thorax')
humerus_virtual_IMU = get_IMU_cal('Humerus')
radius_virtual_IMU = get_IMU_cal('Radius')

# Use the calculated offsets above to update the model's virtual IMUs
apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file)