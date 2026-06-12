from ros import rospy
from std_srvs.srv import Trigger, TriggerResponse

def request_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        service_proxy = rospy.ServiceProxy(service_name, Trigger)
        response = service_proxy()
        return response.success, response.message
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, str(e)

def check_software_version(version_service):
    success, message = request_service(version_service)
    if success:
        rospy.loginfo("Software version: %s" % message)
    else:
        rospy.logerr("Failed to check software version: %s" % message)

def find_file(find_service, file_name):
    success, message = request_service(find_service)
    if success:
        rospy.loginfo("File found: %s" % message)
    else:
        rospy.logerr("Failed to find file: %s" % message)

def transfer_file(transfer_service, file_path):
    success, message = request_service(transfer_service)
    if success:
        rospy.loginfo("File transfer successful: %s" % message)
    else:
        rospy.logerr("File transfer failed: %s" % message)