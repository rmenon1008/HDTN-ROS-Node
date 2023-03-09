#!/usr/bin/env python

import os
import shutil
import json
import subprocess
import logging
import signal
import atexit
import string
import random
import tarfile

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

"""
Important: Only works on Unix systems

Current plan:
Create a utility that can send files using HDTN. It should:

X Be configurable between tx, rx

Sending:
 X Given the path to an item, provide it to BPSendFile
 X Configure HDTN to send the file
    X Generate a config file that enables routing, scheduling
    O Update the contact plan when a refresh is needed
 O Know the current sending status of items

Receiving:
 X When receiving is enabled, allow HDTN to receive items and
    store them in the provided directory
 O Provide a callback for when an item is received or on an error
"""

process_locations = {
    "hdtn_one": "/module/hdtn_one_process/hdtn-one-process",
    "scheduler": "/module/scheduler/hdtn-scheduler",
    "bp_send": "/common/bpcodec/apps/bpsendfile",
    "bp_recv": "/common/bpcodec/apps/bpreceivefile"
}


class HDTN_Base:
    def _setup_common(self, hdtn_root=None):
        if hdtn_root is None:
            hdtn_root = os.environ.get(
                'HDTN_SOURCE_ROOT', os.getcwd() + "/HDTN")
        self.hdtn_root = os.path.abspath(hdtn_root)
        os.environ['HDTN_SOURCE_ROOT'] = self.hdtn_root

        self.build_root = self._find_build_root(hdtn_root)
        self.config_dir = self._create_config_dir()

    def _find_build_root(self, hdtn_root=None):
        if not os.path.exists(hdtn_root):
            logging.error(
                "HDTN source root does not exist. Ensure the provided root"
                "is correct or $HDTN_SOURCE_ROOT is set.")
            raise Exception(
                "HDTN source root does not exist. Ensure $HDTN_SOURCE_ROOT"
                " is set or /HDTN is in the current directory.")
        if not os.path.exists(os.path.join(hdtn_root, "build")):
            raise Exception("HDTN has not been built.")

        return hdtn_root + "/build"

    def _create_config_dir(self):
        dir = os.path.join(os.getcwd(), "hdtnpy_runtime/")
        if not os.path.exists(dir):
            os.makedirs(dir)
        return dir

    def _get_template_config(self, type):
        # Get the template config from ./config_templates
        path = os.path.join(os.path.dirname(__file__),
                            "config_templates", type + ".json")

        with open(path, 'r') as f:
            data = json.load(f)

        return data

    def _start_subprocesses(self):
        logging.info("Starting subprocesses...")

        self._gen_configs()
        self._gen_start_params()

        for key in self.subprocesses:
            start_params = self.subprocesses[key]["start_params"]
            config = self.subprocesses[key]["config"]
            if start_params is None:
                logging.warning(
                    "No start params provided for " + key + ", skipping...")
                continue

            if config is not None:
                with open(self.config_dir + key + "_config.json", "w") as f:
                    json.dump(config, f)
            else:
                logging.debug("No config provided for " + key)

            self.subprocesses[key]["process"] = subprocess.Popen(
                [
                    self.build_root + process_locations[key],
                    *start_params
                ],
                # stdout=subprocess.DEVNULL,
                # stderr=subprocess.DEVNULL,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid
            )

    def _kill_subprocesses(self):
        logging.info("Killing subprocesses...")
        for key in self.subprocesses:
            sp = self.subprocesses[key]["process"]
            if sp is not None:
                sp.kill()

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

        # Start the HDTN subprocesses
        self._start_subprocesses()

        # Set up exit handler
        atexit.register(self.stop)

    def stop(self):
        # Stop the HDTN instance
        logging.info("Stopping HDTN instance...")

        # Stop the subprocesses
        for key in self.subprocesses:
            self._stop_subprocess(self.subprocesses[key]["process"])


class HDTN_Sender(HDTN_Base):
    def __init__(self, settings, hdtn_root=None):
        self.settings = settings
        self._setup_common(hdtn_root)
        self.send_dir = os.path.join(self.config_dir, "send_tar/")

        # Subprocesses
        self.subprocesses = {
            "hdtn_one": {
                "process": None,
                "config": None,
                "start_params": None
            },
            "scheduler": {
                "process": None,
                "config": None,
                "start_params": None
            },
            "bp_send": {
                "process": None,
                "config": None,
                "start_params": None
            }
        }

        # Create the send directory if it doesn't exist
        if not os.path.exists(self.send_dir):
            os.makedirs(self.send_dir)

        # Status
        self.sent_items = []

    def _gen_configs(self):
        logging.info("Generating config files...")

        def hdtn_one_config():
            config = self._get_template_config("hdtn_one")
            config["outductsConfig"]["outductVector"][0]["remoteHostname"] = self.settings["dest_host"]
            return config

        def bp_send_config():
            return self._get_template_config("bp_send")

        self.subprocesses["hdtn_one"]["config"] = hdtn_one_config()
        self.subprocesses["bp_send"]["config"] = bp_send_config()

    def _gen_start_params(self):
        logging.info("Generating start parameters...")

        def hdtn_one_start_params():
            config_arg = '--hdtn-config-file=\"{}\"'.format(
                os.path.join(self.config_dir, "hdtn_one_config.json"))
            return [config_arg]

        def scheduler_start_params():
            config_arg = '--hdtn-config-file=\"{}\"'.format(
                os.path.join(self.config_dir, "hdtn_one_config.json"))
            return ["--contact-plan-file=contactPlanCutThroughMode.json", config_arg]

        def bp_send_start_params():
            config_arg = '--outducts-config-file=\"{}\"'.format(
                os.path.join(self.config_dir, "bp_send_config.json"))

            abs_path = os.path.abspath(self.send_dir)
            logging.info('Sending data from \"{}\"'.format(abs_path))

            send_dir_arg = '--file-or-folder-path=\"{}\"'.format(abs_path)
            return ["--use-bp-version-7",
                    "--max-bundle-size-bytes=4000000",
                    send_dir_arg, "--my-uri-eid={}".format(
                        self.settings["eid"]),
                    "--dest-uri-eid={}".format(self.settings["dest_eid"]),
                    config_arg,
                    "--upload-new-files"]

        self.subprocesses["hdtn_one"]["start_params"] = hdtn_one_start_params()
        self.subprocesses["scheduler"]["start_params"] = scheduler_start_params()
        self.subprocesses["bp_send"]["start_params"] = bp_send_start_params()

    def send_item(self, item_path):
        if not os.path.exists(item_path):
            logging.error("Item does not exist: " + item_path)
            return
        
        # Check if its a tar file already
        if item_path.endswith(".tar"):
            key = os.path.basename(item_path).replace(".tar", "")
            shutil.copy2(item_path, self.send_dir)
        else:
            # Generate a random key
            key = ''.join(random.choices(string.ascii_lowercase + string.digits, k=10))
            # Tar the file or directory
            tar_path = os.path.join(self.send_dir, key + ".tar")
            with tarfile.open(tar_path, "w") as tar:
                tar.add(item_path, arcname=os.path.basename(item_path))

        # Add the item to the send queue
        self.sent_items.append(key)

        return key

class HDTN_Receiver(HDTN_Base):
    def __init__(self, settings, hdtn_root=None, recv_callback=None):
        self.settings = settings
        self._user_recv_callback = recv_callback
        self._setup_common(hdtn_root)
        self.recv_dir = os.path.join(self.config_dir, "recv_tar/")

        # Subprocesses
        self.subprocesses = {
            "hdtn_one": {
                "process": None,
                "config": None,
                "start_params": None
            },
            "bp_recv": {
                "process": None,
                "config": None,
                "start_params": None
            },
            "scheduler": {
                "process": None,
                "config": None,
                "start_params": None
            }
        }

        # Create the recevive directory if it doesn't exist
        if not os.path.exists(self.recv_dir):
            os.makedirs(self.recv_dir)

        # Create the directory monitor
        self.monitor = Observer()
        self.monitor.schedule(
            self.ReceiveDirHandler(self._on_recv),
            self.recv_dir,
            recursive=True
        )
        self.monitor.start()

        # Status
        self.received_items = []

    class ReceiveDirHandler(FileSystemEventHandler):
        def __init__(self, on_recv):
            self.on_recv = on_recv

        def on_closed(self, event):
            try:
                key = os.path.basename(event.src_path).split(".")[0]
                if self.on_recv is not None:
                    self.on_recv(key)
            except Exception as e:
                logging.error("Error in ReceiveDirHandler: " + str(e))

    def _on_recv(self, key):
        logging.info("Received item: " + key)
        self.received_items.append(key)

        if self._user_recv_callback is not None:
            self._user_recv_callback(key, self.recv_dir)

        if self.settings["unpack_after_recv"]:
            # Untar the file
            tar_path = os.path.join(self.recv_dir, key + ".tar")
            with tarfile.open(tar_path, "r") as tar:
                tar.extractall(self.settings["unpack_recv_dir"])
            # Remove the tar file
            os.remove(tar_path)

    def _gen_configs(self):
        logging.info("Generating config files...")

        def hdtn_one_config():
            config = self._get_template_config("hdtn_one")
            return config

        def bp_recv_config():
            return self._get_template_config("bp_recv")

        self.subprocesses["hdtn_one"]["config"] = hdtn_one_config()
        self.subprocesses["bp_recv"]["config"] = bp_recv_config()

    def _gen_start_params(self):
        logging.info("Generating start parameters...")

        def hdtn_one_start_params():
            config_arg = '--hdtn-config-file=\"{}\"'.format(
                os.path.join(self.config_dir, "hdtn_one_config.json"))
            return [config_arg]

        def scheduler_start_params():
            config_arg = '--hdtn-config-file=\"{}\"'.format(
                os.path.join(self.config_dir, "hdtn_one_config.json"))
            return ["--contact-plan-file=contactPlanCutThroughMode.json", config_arg]

        def bp_recv_start_params():
            config_arg = '--inducts-config-file=\"{}\"'.format(
                os.path.join(self.config_dir, "bp_recv_config.json"))
            abs_path = os.path.abspath(self.recv_dir)

            logging.info("Receiving data to \"{}\"".format(abs_path))

            recv_dir_arg = '--save-directory=\"{}\"'.format(abs_path)
            return [recv_dir_arg,
                    "--my-uri-eid={}".format(self.settings["eid"]),
                    config_arg]

        self.subprocesses["hdtn_one"]["start_params"] = hdtn_one_start_params()
        self.subprocesses["scheduler"]["start_params"] = scheduler_start_params()
        self.subprocesses["bp_recv"]["start_params"] = bp_recv_start_params()
