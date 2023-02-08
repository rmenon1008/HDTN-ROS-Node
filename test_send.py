from src.hdtn import HDTN
import time
import logging

settings = {
    "global": {
    },
    "send": {
        "enabled": True,
        "send_dir": "send"
    },
    "receive": {
        "enabled": False,
    },
}

logging.basicConfig(level=logging.DEBUG)

hdtn = HDTN(settings, hdtn_root="../HDTN")

# Start hdtn and poll subprocesses
hdtn.start()
while True:
    print("Status:", hdtn.poll_subprocesses())
    time.sleep(2)