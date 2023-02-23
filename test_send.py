from src.hdtn_new import HDTN_Sender
import time
import logging

settings = {
    "eid": "ipn:1.1",
    "dest_host": "localhost",
    "dest_eid": "ipn:2.1",
    "send_dir": "send"
}

logging.basicConfig(level=logging.DEBUG)

# Setting up an HDTN instance
hdtn = HDTN_Sender(settings, hdtn_root="../HDTN")

# Start hdtn
hdtn.start()

time.sleep(2)

hdtn.send_item("TEST_DIR")

time.sleep(20)

hdtn.send_item("h_index.py")

while True:
    time.sleep(2)