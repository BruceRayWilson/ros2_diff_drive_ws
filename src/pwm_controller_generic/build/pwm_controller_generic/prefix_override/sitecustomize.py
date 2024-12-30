import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ros2_diff_drive_ws/src/pwm_controller_generic/install/pwm_controller_generic'
