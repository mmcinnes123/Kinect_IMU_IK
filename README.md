This repo has been made to run IMU-based Inverse Kinematics with body orientations measured by the Kinect markerless camera. 
Input is a csv file with the Kinect body orientations in quaternions, which is converted into an .sto so that OpenSim can read it. 
Output is a .mot coordinates files which is the result of the inverse kinematics. 
Included is a calibration script showing how the calibrated model was created, simply by specifying virtual IMUs corresponding to each mody body, based on how they're specified in the Kinect frames (https://learn.microsoft.com/en-us/azure/kinect-dk/coordinate-systems). 
