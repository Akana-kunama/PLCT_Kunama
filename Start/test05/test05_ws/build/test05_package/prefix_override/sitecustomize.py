import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kunama/Desktop/PLCT/test05/test05_ws/install/test05_package'
