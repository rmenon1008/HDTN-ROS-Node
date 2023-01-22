import os
import shutil
import json

"""
Current plan:

Create a utility that can send files using HDTN. It should:
 - Given the path to an item, provide it to BPSendFile
 - Configure HDTN to send the file
    - Generate a config file that enables routing, scheduling
    - Update the contact plan when a refresh is needed
 - Know the current sending status of items

 Eventually, it should:
    - Be configurable between tx, rx and transfer modes
"""


class HDTN:
    def __init__(self, config):
        self.build_root = self._find_build_root()
        self.config_dir = self._create_config_dir()

        # self.mode = "send"
        

    def _find_build_root(self):
        hdtn_root = os.environ.get('HDTN_SOURCE_ROOT', os.getcwd() + "/HDTN")
        if not os.path.exists(hdtn_root):
            raise Exception(
                "HDTN source root does not exist. Ensure $HDTN_SOURCE_ROOT is set or /HDTN is in the current directory.")
        if not os.path.exists(hdtn_root + "/build"):
            raise Exception("HDTN has not been built.")

        return hdtn_root + "/build"

    def _create_config_dir(self):
        dir = 'hdtn_config'
        if os.path.exists(dir):
            shutil.rmtree(dir)
        os.makedirs(dir)

        return os.getcwd() + "/" + dir

    def gen_contact_plan(self, contact_plan):
        """
        Sets the contact plan for the HDTN instance
        :param contact_plan: A dictionary that follows the HDTN contact plan format
        """
        with open(self.config_dir + "contact_plan.json", "w") as f:
            json.dump(self.contact_plan, f)

    def gen_config(self, config):
        """
        Generates a config file for the HDTN instance
        :param config: A dictionary that follows the HDTN config format
        """
        with open(self.config_dir + "config.json", "w") as f:
            json.dump(config, f)

    def start(self):
        pass

    def stop(self):
        pass

    def send_item(self, item_path):
        """
        Sends an item to the HDTN instance
        :param item_path: The absolute path to a file or directory to be copied to the remote instance
        """

        # if self.mode != "send":
        #     raise Exception("Must be in send mode to send items")
        # if not os.path.exists(item_path):        #     raise Exception("Item path does not exist")


    
    def receive_item(self, item_path):
        """
        Receives an item from the HDTN instance
        :param item_path: The absolute path to a directory to copy the item to
        """

        # if self.mode != "receive":
        #     raise Exception("Must be in receive mode to receive items")
        # if not os.path.exists(item_path):
        #     raise Exception("Item path does not exist")
