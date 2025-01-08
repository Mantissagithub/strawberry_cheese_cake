import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pradheep/Documents/strawberry_cheese_cake/irc_autonomous_ws/install/autonomous_stack'
