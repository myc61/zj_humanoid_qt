import unittest
from src.ros.connection import ROSConnection
from src.ros.version_check import check_version
from src.ros.services import request_service

class TestROSModule(unittest.TestCase):

    def setUp(self):
        self.ros_connection = ROSConnection()
        self.ros_connection.connect()

    def tearDown(self):
        self.ros_connection.disconnect()

    def test_check_version(self):
        version = check_version(self.ros_connection)
        self.assertIsNotNone(version)
        self.assertIsInstance(version, str)

    def test_request_service(self):
        response = request_service(self.ros_connection, 'some_service')
        self.assertIsNotNone(response)
        self.assertTrue(response['success'])

if __name__ == '__main__':
    unittest.main()