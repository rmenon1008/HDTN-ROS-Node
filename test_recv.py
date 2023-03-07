from src.hdtn import HDTN_Receiver
import time
import logging

logging.basicConfig(level=logging.INFO)

settings = {
    "eid": "ipn:2.1",
    "unpack_after_recv": True,
    "unpack_recv_dir": "received_items",
}

def on_item_received(item):
    print("Received item:", item)

# Setting up an HDTN instance
hdtn = HDTN_Receiver(settings, hdtn_root="../HDTN", recv_callback=on_item_received)

# Start hdtn
hdtn.start()

# Busy wait
while True:
    time.sleep(2)