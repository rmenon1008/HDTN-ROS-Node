import os
import shutil
import json
import subprocess

"""
Current plan:

Create a utility that can send files using HDTN. It should:

- Be configurable between tx, rx

Sending:
 - Given the path to an item, provide it to BPSendFile
 - Configure HDTN to send the file
    - Generate a config file that enables routing, scheduling
    - Update the contact plan when a refresh is needed
 - Know the current sending status of items

Receiving:
    - When receiving is enabled, allow HDTN to receive items and
      store them in the provided directory
    - Provide a callback for when an item is received or on an error

Transfer:
    - When transfer is enabled, allow HDTN to transfer items
    - Provide a callback for when an item is transferred or on an error
"""

settings = {
    "global": {
        "log_level": "info",
        "log_file": "hdtn.log",
    },
    "send": {
        "enabled": True,
        "source_dir": "send",
    },
    "receive": {
        "enabled": True,
        "target_dir": "received",
    },
}

class HDTN:
    def __init__(self, settings, hdtn_root=None):
        # Ensure the HDTN is available and built
        self.build_root = self._find_build_root(hdtn_root)
        self.config_dir = self._create_config_dir()
        
        # Class parameters
        self.settings = settings

        # Subprocesses
        self.hdtn_sp = None
        self.bp_send_sp = None
        self.bp_recv_sp = None

    def _find_build_root(self, hdtn_root=None):
        if hdtn_root is None:
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

    def _set_contact_plan(self, contact_plan):
        """
        Saves the contact plan for the HDTN instance to the config directory
        :param contact_plan: A dictionary that follows the HDTN contact plan format
        """
        if contact_plan is None:
            raise Exception("Contact plan does not exist")
        with open(self.config_dir + "contact_plan.json", "w") as f:
            json.dump(contact_plan, f)

    def _set_config_files(self, configs):
        """
        Saves config files for the HDTN instances to the config directory
        :param config: An object containing configs for one or more subprocesses (HDTN One, BPSendFile, BPRecvFile)
        """
        if configs is None:
            raise Exception("No configs provided")
        
        if "hdtn_one" in configs:
            with open(self.config_dir + "hdtn_one_config.json", "w") as f:
                json.dump(configs.hdtn_one, f)
        
        if "bp_send" in configs:
            with open(self.config_dir + "bp_send_config.json", "w") as f:
                json.dump(configs.bp_send, f)

        if "bp_recv" in configs:
            with open(self.config_dir + "bp_recv_config.json", "w") as f:
                json.dump(configs.bp_recv, f)


    def _gen_hdtn_one_config(self, settings):
        # TODO
        return settings

    def _gen_bp_send_config(self, settings):
        # TODO
        return settings

    def _gen_bp_recv_config(self, settings):
        # TODO
        return settings

    def start(self):
        # Some translation step to convert the settings into a config file
        hdtn_config = self._gen_hdtn_one_config(self.settings)

        # Generate the config files
        configs = {
            "hdtn_one": self._gen_hdtn_one_config(),
        }
        if self.settings["send"]["enabled"]:
            configs["bp_send"] = self._gen_bp_send_config()
        if self.settings["receive"]["enabled"]:
            configs["bp_recv"] = self._gen_bp_recv_config()
            
        self._set_config_files(configs)

        # Start the HDTN one instance
        self.hdtn_sp = subprocess.Popen([self.build_root + "/src/hdtn/hdtn"])

        if self.settings["send"]["enabled"]:
            self.bp_send_sp = subprocess.Popen([self.build_root + "/src/hdtn/bp_send_file"])
        
        if self.settings["receive"]["enabled"]:
            self.bp_recv_sp = subprocess.Popen([self.build_root + "/src/hdtn/bp_recv_file"])
        
        # Set up callbacks for sending and receiving items
        pass

    def get_status(self):
        # Get the status of the HDTN instances
        hdtn_one_status = self.hdtn_sp.poll()
        bp_send_status = self.bp_send_sp.poll()
        bp_recv_status = self.bp_recv_sp.poll()

        return {
            "hdtn_one": hdtn_one_status,
            "bp_send": bp_send_status,
            "bp_recv": bp_recv_status,
        }

    def stop(self):
        # Stop the HDTN instance

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
