from src.hdtn import HDTN
import time
import logging
import os

settings = {
    "global": {
    },
    "send": {
        "enabled": False,
    },
    "receive": {
        "enabled": True,
        "receive_dir": "received"
    },
}

logging.basicConfig(level=logging.DEBUG)

hdtn = HDTN(settings, hdtn_root="../HDTN")

# Check if receive folder exists and create it if not
if not os.path.exists(settings['receive']['receive_dir']):
    print("Creating receive directory")
    os.makedirs(settings['receive']['receive_dir'])

# Start hdtn and poll subprocesses
hdtn.start()
while True:
    print("Status:", hdtn.poll_subprocesses())
    time.sleep(2)