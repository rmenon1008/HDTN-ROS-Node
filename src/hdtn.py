import os
import shutil
import json
import subprocess
import logging
import signal

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
"""


class HDTN:
    def __init__(self, settings, hdtn_root=None):
        # Ensure the HDTN is available and built
        # Set up a folder for configuration files
        if hdtn_root is None:
            hdtn_root = os.environ.get(
                'HDTN_SOURCE_ROOT', os.getcwd() + "/HDTN")
        self.hdtn_root = os.path.abspath(hdtn_root)
        os.environ['HDTN_SOURCE_ROOT'] = self.hdtn_root

        self.build_root = self._find_build_root(hdtn_root)
        self.config_dir = self._create_config_dir()

        # Class parameters
        self.settings = settings

        # Subprocesses
        self.hdtn_sp = None
        self.bp_send_sp = None
        self.bp_recv_sp = None

    def _find_build_root(self, hdtn_root=None):
        if not os.path.exists(hdtn_root):
            logging.error("HDTN source root does not exist. Ensure $HDTN_SOURCE_ROOT is set or /HDTN is in the current directory.")
            raise Exception(
                "HDTN source root does not exist. Ensure $HDTN_SOURCE_ROOT is set or /HDTN is in the current directory.")
        if not os.path.exists(os.path.join(hdtn_root, "build")):
            raise Exception("HDTN has not been built.")

        return hdtn_root + "/build"

    def _create_config_dir(self):
        dir = os.path.join(self.build_root, "config_hdtnpy/")
        if os.path.exists(dir):
            shutil.rmtree(dir)
        os.makedirs(dir)

        return dir

    def _set_contact_plan(self, contact_plan):
        """
        Saves the contact plan for the HDTN instance to the config directory
        :param contact_plan: A dictionary that follows the HDTN contact plan format
        """
        if contact_plan is None:
            logging.error("Contact plan does not exist")
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
                json.dump(configs["hdtn_one"], f)

        if "bp_send" in configs:
            with open(self.config_dir + "bp_send_config.json", "w") as f:
                json.dump(configs["bp_send"], f)

        if "bp_recv" in configs:
            with open(self.config_dir + "bp_recv_config.json", "w") as f:
                json.dump(configs["bp_recv"], f)

    def _get_template_config(self, type):
        # Get the template config from ./config_templates
        path = os.path.join(os.path.dirname(__file__),
                            "config_templates", type + ".json")

        with open(path, 'r') as f:
            data = json.load(f)

        return data

    def _gen_configs(self):
        logging.info("Generating config files...")

        def hdtn_one_config():
            return self._get_template_config("hdtn_one")

        def bp_send_config():
            return self._get_template_config("bp_send")

        def bp_recv_config():
            return self._get_template_config("bp_recv")

        configs = {
            "hdtn_one": hdtn_one_config(),
        }
        if self.settings["send"]["enabled"]:
            configs["bp_send"] = bp_send_config()
        if self.settings["receive"]["enabled"]:
            configs["bp_recv"] = bp_recv_config()
        return configs

    def _gen_start_params(self):
        logging.info("Generating start parameters...")

        def hdtn_one_start_params():
            # TODO
            config_arg = '--hdtn-config-file={}'.format(
                os.path.join(self.config_dir, "hdtn_one_config.json"))
            return [config_arg]

        def bp_send_start_params():
            # TODO
            config_arg = '--outducts-config-file={}'.format(
                os.path.join(self.config_dir, "bp_send_config.json"))
            send_dir_arg = "--file-or-folder-path={}".format(
                os.path.join(self.hdtn_root, self.settings["send"]["send_dir"]))
            return ["--use-bp-version-7", "--max-bundle-size-bytes=4000000", send_dir_arg, "--my-uri-eid=ipn:1.1", "--dest-uri-eid=ipn:2.1", config_arg]

        def bp_recv_start_params():
            # TODO
            config_arg = '--inducts-config-file={}'.format(
                os.path.join(self.config_dir, "bp_recv_config.json"))
            return ["--save-directory=received", "--my-uri-eid=ipn:2.1", config_arg]

        params = {
            "hdtn_one": hdtn_one_start_params(),
        }
        if self.settings["send"]["enabled"]:
            params["bp_send"] = bp_send_start_params()
        if self.settings["receive"]["enabled"]:
            params["bp_recv"] = bp_recv_start_params()
        return params

    def _start_subprocesses(self, start_params):
        logging.info("Starting subprocesses...")

        self.hdtn_sp = subprocess.Popen(
            [self.build_root + "/module/hdtn_one_process/hdtn-one-process", *start_params["hdtn_one"]])
        if self.settings["send"]["enabled"]:
            self.bp_send_sp = subprocess.Popen(
                [self.build_root + "/common/bpcodec/apps/bpsendfile", *start_params["bp_send"]])
        if self.settings["receive"]["enabled"]:
            self.bp_recv_sp = subprocess.Popen(
                [self.build_root + "/common/bpcodec/apps/bpreceivefile", *start_params["bp_recv"]])

    def _kill_subprocesses(self):
        logging.info("Killing subprocesses...")

        if self.hdtn_sp is not None:
            self.hdtn_sp.kill()
        if self.bp_send_sp is not None:
            self.bp_send_sp.kill()
        if self.bp_recv_sp is not None:
            self.bp_recv_sp.kill()

    def _stop_subprocess(self, sp):
        try:
            if sp is not None:
                logging.info("Stopping subprocess " + str(sp))
                sp.send_signal(signal.SIGINT)
                sp.wait(timeout=5)
        except subprocess.TimeoutExpired:
            logging.warning("Subprocess " + str(sp) + " did not stop cleanly")
            sp.kill()

    def start(self):
        logging.info("Starting HDTN instance...")
        # Generate the config files
        configs = self._gen_configs()
        self._set_config_files(configs)
        start_params = self._gen_start_params()

        # Start the HDTN subprocesses
        self._start_subprocesses(start_params)

        # Set up callbacks for sending and receiving items
        pass

    def poll_subprocesses(self):
        # Get the status of the HDTN instances
        stat = {
            "hdtn_one": None,
            "bp_send": None,
            "bp_recv": None,
        }

        if self.hdtn_sp is not None:
            stat["hdtn_one"] = self.hdtn_sp.poll()

        if self.bp_send_sp is not None:
            stat["bp_send"] = self.bp_send_sp.poll()

        if self.bp_recv_sp is not None:
            stat["bp_recv"] = self.bp_recv_sp.poll()

        return stat

    def stop(self):
        # Stop the HDTN instance
        logging.info("Stopping HDTN instance...")
        self._stop_subprocess(self.hdtn_sp)
        self._stop_subprocess(self.bp_send_sp)
        self._stop_subprocess(self.bp_recv_sp)
