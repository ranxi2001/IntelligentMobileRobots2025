import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/IntelligentMobileRobots2025/work3/install/lms_data'
