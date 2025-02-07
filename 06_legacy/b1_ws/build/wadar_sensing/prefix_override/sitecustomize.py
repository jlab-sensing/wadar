import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ericdvet/jlab/wadar/b1_ws/install/wadar_sensing'
