import unittest 
import re

class SimpleTest(unittest.TestCase): 
  
    # Returns True or False.  
    def test_file(self):         
        pattern = re.compile(r'file://(.*?)', re.S)
        new_value = re.sub(pattern,
            lambda x: x.group(1),
            "file://dddd")
        print(new_value)
            
    def test_package(self):         
        pattern = re.compile(r'file://(.*?)', re.S)
        new_value = re.sub(pattern,
            lambda x: x.group(1),
            "package://dddd")
        print(new_value)

    def test_search(self):         
        pattern = re.compile(r'file://(.*?)', re.S)
        new_value = pattern.search("file://dddd")
            
        print(new_value.span())

if __name__ == '__main__': 
    unittest.main() 
