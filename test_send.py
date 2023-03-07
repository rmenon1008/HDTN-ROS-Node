from src.hdtn import HDTN_Sender
import time
import logging

logging.basicConfig(level=logging.INFO)

settings = {
    "eid": "ipn:1.1",
    "dest_host": "localhost",
    "dest_eid": "ipn:2.1",
}

def on_item_sent(item):
    print("Sent item:", item)

# Setting up an HDTN instance
hdtn = HDTN_Sender(settings, hdtn_root="../HDTN", send_callback=on_item_sent)

# Start hdtn
hdtn.start()

# Send a file
hdtn.send_item("test_files/test.txt")
time.sleep(5)

# Send a directory
hdtn.send_item("test_files/test_dir")

# Busy wait
while True:
    time.sleep(2)