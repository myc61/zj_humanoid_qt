import rospy
from std_msgs.msg import String

def check_software_version():
    try:
        # Assuming there is a ROS service that provides the software version
        rospy.wait_for_service('get_software_version')
        get_version = rospy.ServiceProxy('get_software_version', String)
        version = get_version().data
        return version
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

if __name__ == "__main__":
    rospy.init_node('version_check_node')
    version = check_software_version()
    if version:
        rospy.loginfo("Software version: %s" % version)
    else:
        rospy.loginfo("Failed to retrieve software version.")