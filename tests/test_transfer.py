import unittest
from src.transfer.upload import upload_file
from src.transfer.download import download_file

class TestFileTransfer(unittest.TestCase):

    def test_upload_file(self):
        # Test the upload functionality
        result = upload_file('test_file.txt', '/robot/path/test_file.txt')
        self.assertTrue(result)

    def test_download_file(self):
        # Test the download functionality
        result = download_file('/robot/path/test_file.txt', 'downloaded_test_file.txt')
        self.assertTrue(result)

if __name__ == '__main__':
    unittest.main()