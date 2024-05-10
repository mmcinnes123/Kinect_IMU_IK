
import opensim as osim
from scipy.spatial.transform import Rotation as R



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
    humerus_imu_weight = osim.OrientationWeight('humerus_r_imu', 0.1)
    radius_imu_weight = osim.OrientationWeight('radius_r_imu', 1.0)
    print('WARNING: Humerus IMU weight set to 0.1')
    imuIK.upd_orientation_weights().cloneAndAppend(thorax_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(humerus_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(radius_imu_weight)

    # Run IK
    imuIK.run(visualize_tracking)

    # Update the settings .xml file
    imuIK.printToXML(results_directory + "\\" + IMU_IK_settings_file)