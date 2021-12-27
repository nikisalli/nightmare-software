import rospy
from enum import Enum


class loglevel(Enum):
    INFO = 0
    WARN = 1
    ERROR = 2
    FATAL = 3


def printlog(msg, level=loglevel.INFO):
    header = f"[{rospy.get_name()}] "
    new_msg = header + msg
    if level == loglevel.INFO:
        rospy.loginfo(new_msg)
    elif level == loglevel.WARN:
        rospy.logwarn(new_msg)
    elif level == loglevel.ERROR:
        rospy.logerr(new_msg)
    elif level == loglevel.FATAL:
        rospy.logfatal(new_msg)
