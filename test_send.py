from src.hdtn import HDTN_Sender
import time
import logging

settings = {
    "eid": "ipn:1.1",
    "dest_host": "localhost",
    "dest_eid": "ipn:2.1",
}

logging.basicConfig(level=logging.INFO)

# Setting up an HDTN instance
hdtn = HDTN_Sender(settings, hdtn_root="../HDTN")

# Start hdtn
hdtn.start()

time.sleep(2)

hdtn.send_item("srv")

time.sleep(20)

hdtn.send_item("kill.sh")

while True:
    time.sleep(2)
    print("Sent items:", hdtn.sent_items)