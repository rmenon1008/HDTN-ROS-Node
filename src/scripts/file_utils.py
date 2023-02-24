import os
import shutil

"""
file utils that are shared between the send and receive nodes
"""


def clear_dir(path):
    """erases contents of directory"""
    for root, dirs, files in os.walk(path, topdown=False):
        for name in files:
            os.remove(os.path.join(root, name))
        for name in dirs:
            os.rmdir(os.path.join(root, name))


def copy_directory(src, dst):
    """
    moves contents of src directory to send directory
    :param req: msg which contains directory to move contents of.
    :return: None
    """
    shutil.copy(src, dst)


def setup_directory(path):
    """
    creates directory for htdn node, if directory already exists, it is cleared
    :param path: file path for new directory
    :return: None
    """
    if not os.path.exists(path):
        print("Creating directory " + path)
        os.makedirs(path)
    else:
        print("Directory already exists " + path)
    clear_dir(path)