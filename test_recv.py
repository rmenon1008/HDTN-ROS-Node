from src.scripts.hdtn import HDTN
import time
import logging
import os

settings = {
    "global": {
        "eid": "ipn:2.1",
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
    print("Status:", )
    time.sleep(2)