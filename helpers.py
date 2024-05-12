
import opensim as osim
from scipy.spatial.transform import Rotation as R
import pandas as pd
import numpy as np


# Read quaternion data from a csv file, given the column headers
def read_data_frame_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=0, sep=",")
    # Make seperate data_out frames
    humerus_quats = df.filter(["Shoulder W", "Shoulder X", "Shoulder Y", "Shoulder Z"], axis=1)
    clavicle_quats = df.filter(["Clavicle W", "Clavicle X", "Clavicle Y", "Clavicle Z"], axis=1)
    thorax_quats = df.filter(["Thorax W", "Thorax X", "Thorax Y", "Thorax Z"], axis=1)
    radius_quats = df.filter(["Elbow W", "Elbow X", "Elbow Y", "Elbow Z"], axis=1)
    return thorax_quats, clavicle_quats, humerus_quats, radius_quats


# Write new data_out to APDM file template
def write_to_APDM_csv(df_1, df_2, df_3, df_4, template_file, output_dir, csv_file_name):

    # Make columns of zeros
    N = len(df_1)
    zeros_25_df = pd.DataFrame(np.zeros((N, 25)))
    zeros_11_df = pd.DataFrame(np.zeros((N, 11)))
    zeros_2_df = pd.DataFrame(np.zeros((N, 2)))

    # Make a dataframe with zeros columns inbetween the data_out
    IMU_and_zeros_df = pd.concat(
        [zeros_25_df, df_1, zeros_11_df, df_2, zeros_11_df, df_3, zeros_11_df, df_4, zeros_2_df], axis=1)

    # Read in the APDM template and save as an array
    with open(template_file, 'r') as file:
        template_df = pd.read_csv(file, header=0)
        template_array = template_df.to_numpy()

    # Concatenate the IMU_and_zeros and the APDM template headings
    IMU_and_zeros_array = IMU_and_zeros_df.to_numpy()
    new_array = np.concatenate((template_array, IMU_and_zeros_array), axis=0)
    new_df = pd.DataFrame(new_array)

    # Add the new dataframe into the template
    new_df.to_csv(output_dir + "\\" + csv_file_name, mode='w', index=False, header=False, encoding='utf-8', na_rep='nan')


# Use OpenSim's APDMDataReader tool to convert APDM csv file to sto file
def APDM_2_sto_Converter(APDM_settings_file, input_file, output_file):

    # Build an APDM Settings Object
    # Instantiate the Reader Settings Class
    APDMSettings = osim.APDMDataReaderSettings(APDM_settings_file)
    # Instantiate an APDMDataReader
    APDM = osim.APDMDataReader(APDMSettings)

    # Read in table of movement data_out from the specified IMU file
    table = APDM.read(input_file)
    # Get Orientation Data as quaternions
    quatTable = APDM.getOrientationsTable(table)

    # Write to file
    osim.STOFileAdapterQuaternion.write(quatTable, output_file)



# Function to read .csv file with quaternion data and save it to .sto file
# (This uses OpenSim's 'APDM_2_sto_Converter', so we first save the data in the format of an APDM .csv data file
# as a mid-point between the input .csv and the .sto.)
def write_movements_and_calibration_stos(file_path, output_dir, APDM_template_csv, APDM_settings_file):

    # Read data from the .csv file, save as dataframes
    thorax_quats, clavicle_quats, humerus_quats, radius_quats = read_data_frame_from_file(file_path)

    # Write the data to an APDM-style .csv file
    csv_file_name = 'APDM_Kinect_Body_Quats_all.csv'
    write_to_APDM_csv(thorax_quats, clavicle_quats, humerus_quats, radius_quats,
                      APDM_template_csv, output_dir, csv_file_name)

    # Write the data to an .sto using OpenSim APDM converter tool
    sto_file_name = 'Kinect_Body_Quats_all.sto'
    APDM_2_sto_Converter(APDM_settings_file, input_file=output_dir + "\\" + csv_file_name,
                         output_file=output_dir + "\\" + sto_file_name)







# This function calculates the IMU offset required which is equivalent to relying on 'manual alignment'
# The only reason we need to apply an offset (and not just have 0 offset) is because the IMU axis names 'xyz' don't
# match the names of the body axes, so are only rotated in multiples of 90degrees
def get_IMU_cal_MANUAL(which_body):

    if which_body == "Thorax":
        virtual_IMU = R.from_euler('XYZ', [-90, -90, 0], degrees=True)

    elif which_body == "Humerus":
        virtual_IMU = R.from_euler('XYZ', [90, 90, 0], degrees=True)

    elif which_body == "Radius":
        virtual_IMU = R.from_euler('XYZ', [0, 0, 90], degrees=True)

    else:
        print("Which_body input wasn't 'Thorax', 'Humerus', or 'Radius'")

    return virtual_IMU





# A function which takes an uncalibrated model (with IMUs already associated with each body)
# and inputs an euler orientation offset which defines the virtual IMU offset relative to the bodies
def apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, results_dir):

    model = osim.Model(model_file)

    def set_IMU_transform(virtual_IMU_R, body_name, imu_name):

        # Define a new OpenSim rotation from the scipy rotation
        mat_from_scipy = virtual_IMU_R.as_matrix()
        rot = osim.Rotation(osim.Mat33(mat_from_scipy[0,0], mat_from_scipy[0,1], mat_from_scipy[0,2],
                                       mat_from_scipy[1,0], mat_from_scipy[1,1], mat_from_scipy[1,2],
                                       mat_from_scipy[2,0], mat_from_scipy[2,1], mat_from_scipy[2,2]))

        # Apply the rotation to the IMU in the model
        IMU_frame = model.getBodySet().get(body_name).getComponent(imu_name)    # Get the exsisting phyiscal offset frame of the IMU
        trans_vec = IMU_frame.getOffsetTransform().T()    # Get the existing translational offset of the IMU frame
        transform = osim.Transform(rot, osim.Vec3(trans_vec))   # Create an opensim transform from the rotation and translation
        IMU_frame.setOffsetTransform(transform)  # Update the IMU frame transform

    set_IMU_transform(thorax_virtual_IMU, body_name='thorax', imu_name='thorax_imu')
    set_IMU_transform(humerus_virtual_IMU, body_name='humerus_r', imu_name='humerus_r_imu')
    set_IMU_transform(radius_virtual_IMU, body_name='radius_r', imu_name='radius_r_imu')

    model.setName("IMU_Calibrated_das")
    model.printToXML("Calibrated_" + model_file)


def run_osim_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file,
               sensor_to_opensim_rotations, results_directory, trim_bool,
               start_time, end_time, IK_output_file_name, visualize_tracking):

    # Instantiate an InverseKinematicsTool
    imuIK = osim.IMUInverseKinematicsTool(IMU_IK_settings_file)

    # Set tool properties
    imuIK.set_model_file(calibrated_model_file)
    imuIK.set_orientations_file(orientations_file)
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuIK.set_results_directory(results_directory)
    if trim_bool == True:
        imuIK.set_time_range(0, start_time)
        imuIK.set_time_range(1, end_time)
    imuIK.setOutputMotionFileName(IK_output_file_name)

    # Set IMU weights
    thorax_imu_weight = osim.OrientationWeight('thorax_imu', 1.0)
    humerus_imu_weight = osim.OrientationWeight('humerus_r_imu', 1.0)
    radius_imu_weight = osim.OrientationWeight('radius_r_imu', 1.0)
    imuIK.upd_orientation_weights().cloneAndAppend(thorax_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(humerus_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(radius_imu_weight)

    # Run IK
    imuIK.run(visualize_tracking)

    # # Save the settings .xml file for reference
    # imuIK.printToXML(results_directory + "\\" + IMU_IK_settings_file)
