from calibration_toolbox.axxb import AXXBCalibrator
import argparse

AXXBCalib = AXXBCalibrator('EBCB')
parser = argparse.ArgumentParser(description='Calibration Node')
parser.add_argument('--data_path', type=str, required=True, help='Path to save calibration data')
args = parser.parse_args()
AXXBCalib.load_xforms(args.data_path)

cam2ee = AXXBCalib.axxb()

AXXBCalib.test()
AXXBCalib.write_pose_file()