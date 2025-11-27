import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/trainee/workspace/trainee_ws_pinguin/src/install/webots_packed'
