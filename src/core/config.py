class Config:
    def __init__(self):
        self.ros_host = "localhost"
        self.ros_port = 11311
        self.default_file_path = "/home/user/robot_files"
        self.log_level = "INFO"
        self.timeout = 10  # seconds

    def get_ros_connection_string(self):
        return f"{self.ros_host}:{self.ros_port}"

    def get_default_file_path(self):
        return self.default_file_path

    def get_log_level(self):
        return self.log_level

    def get_timeout(self):
        return self.timeout