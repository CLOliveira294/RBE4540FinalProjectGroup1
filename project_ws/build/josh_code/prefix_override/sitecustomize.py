import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/josh_ym_lee/RBE4540FinalProjectGroup1/project_ws/install/josh_code'
