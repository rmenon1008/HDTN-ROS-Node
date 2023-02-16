from src.hdtn import HDTN
import time
import logging

settings = {
    "global": {
        "eid": "ipn:1.1",
        "logging": True
    },
    "send": {
        "enabled": True,
        "dest_host": "localhost",
        "dest_eid": "ipn:2.1",
        "send_dir": "send"
    },
    "receive": {
        "enabled": False,
    },
}

logging.basicConfig(level=logging.DEBUG)

# Setting up an HDTN instance
hdtn = HDTN(settings, hdtn_root="../HDTN")

# Start hdtn
hdtn.start()

# Monitor the status
while True:
    print("Status:", hdtn.get_stats())
    time.sleep(2)
    