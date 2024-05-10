
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
