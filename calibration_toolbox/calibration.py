from calibration_toolbox.axxb import AXXBCalibrator
import argparse

parser = argparse.ArgumentParser(description='Calibration Node')
parser.add_argument('--data_path', type=str, required=True, help='Path to save calibration data')
parser.add_argument('--calib_type', type=str, choices=['EBCB', 'EBME', 'EH'], required=True, help='Type of calibration')
args = parser.parse_args()

AXXBCalib = AXXBCalibrator(args.calib_type)
AXXBCalib.load_xforms(args.data_path)

cam2ee = AXXBCalib.axxb()

AXXBCalib.test()
AXXBCalib.write_pose_file()