import unittest
from src.files.finder import FileFinder

class TestFileFinder(unittest.TestCase):

    def setUp(self):
        self.finder = FileFinder()

    def test_find_file_existing(self):
        result = self.finder.find_file("existing_file.txt")
        self.assertTrue(result)

    def test_find_file_non_existing(self):
        result = self.finder.find_file("non_existing_file.txt")
        self.assertFalse(result)

    def test_find_file_with_empty_name(self):
        result = self.finder.find_file("")
        self.assertFalse(result)

if __name__ == '__main__':
    unittest.main()