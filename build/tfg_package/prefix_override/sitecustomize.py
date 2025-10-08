import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aalbeerto-02/tfg_girs_ws/install/tfg_package'
