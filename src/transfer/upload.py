import os
import rospy
from std_msgs.msg import String

def upload_file(file_path, robot_path):
    if not os.path.isfile(file_path):
        rospy.logerr("File does not exist: {}".format(file_path))
        return False

    try:
        # Assuming we have a ROS service to handle file uploads
        rospy.wait_for_service('upload_file_service')
        upload_service = rospy.ServiceProxy('upload_file_service', String)
        
        with open(file_path, 'rb') as file:
            file_data = file.read()
            response = upload_service(robot_path, file_data)
            rospy.loginfo("Upload response: {}".format(response))
            return True
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False
    except Exception as e:
        rospy.logerr("An error occurred: {}".format(e))
        return False