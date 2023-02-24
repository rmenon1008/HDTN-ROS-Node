import os

"""
file utils that are shared between the send and receive nodes
"""

def clear_dir(path):
    """erases contents of directory"""
    return 0



def move_directory(src, dest):
    """
    moves contents of src directory to send directory
    :param req: msg which contains directory to move contents of.
    :return: None
    """

    return 0

def setup_directory(path):
    """
    creates directory for htdn node
    :param path: file path for new directory
    :return: None
    """
    if not os.path.exists(path):
        print("Creating directory " + path)
        os.makedirs(path)
    else:
        print("Directory already exists " + path)