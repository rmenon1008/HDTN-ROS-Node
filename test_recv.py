from src.hdtn_new import HDTN_Receiver
import time
import logging

settings = {
    "eid": "ipn:2.1",
    "recv_dir": "received"
}

logging.basicConfig(level=logging.DEBUG)

# Setting up an HDTN instance
hdtn = HDTN_Receiver(settings, hdtn_root="../HDTN")

# Start hdtn
hdtn.start()

# Monitor the status
while True:
    time.sleep(2)