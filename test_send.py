from src.hdtn import HDTN
import time
import logging
import atexit

settings = {
    "global": {
        "log_level": "info",
        "log_file": "hdtn.log"
    },
    "send": {
        "enabled": True,
        "send_dir": "sending"
    },
    "receive": {
        "enabled": False,
        "target_dir": "received"
    },
}

logging.basicConfig(level=logging.INFO)

hdtn = HDTN(settings, hdtn_root="../HDTN")
atexit.register(hdtn.stop)

hdtn.start()

while True:
    time.sleep(1)
    print("Status:", hdtn.poll_subprocesses())

# time.sleep(10)
# atexit.unregister(hdtn.stop)
# hdtn.stop()