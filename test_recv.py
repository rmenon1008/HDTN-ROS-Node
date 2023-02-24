from src.hdtn import HDTN_Receiver
import time
import logging

settings = {
    "eid": "ipn:2.1",
    "unpack_after_recv": True,
    "unpack_recv_dir": "received_items",
}

logging.basicConfig(level=logging.INFO)

# Setting up an HDTN instance
hdtn = HDTN_Receiver(settings, hdtn_root="../HDTN")

# Start hdtn
hdtn.start()

# Monitor the status
while True:
    time.sleep(2)
    print("Received items:", hdtn.received_items)