#!/usr/bin/env python
from __future__ import print_function
#from hdtn import HDTN_Receiver
from hdtn_fake import HDTN_Receiver
import rospy
import logging
from hdtn_node.srv import SendFileTo, SendFileToResponse
from hdtn_node.msg import HdtnRecStatus

REFRESH_RATE = 10

settings = settings = {
    "eid": "ipn:2.1",
    "unpack_after_recv": True,
    "unpack_recv_dir": "received_items",
}

class RecNode:
    """
    This should initialize the HDTN with rec config.

    Service: SendData: input is a file path, contents of receive dir is copied and moved to output path.
    Publisher: Status of HDTN node
    """


    def hdtn_init(self):
        """
        initializes the HDTN node in receive mode
        :return: None
        """
        self.hdtn = HDTN_Receiver(settings, hdtn_root="../HDTN", rec_callback=self.add_rec_file)
        self.hdtn.start


    def get_data(self, req):
        """
        copies contents of receive directory to specified directory
        :param req: msg which contains dst directory
        :return: None
        """
        dst = req.filepath
        print(dst)
        # self.hdtn.settings["unpack_recv_dir"] = "received_items"
        return SendFileToResponse()

    def hdtn_status(self):
        """
        publishes files recieved over hdtn
        :return: None
        """
        num_files = len(self.rec_files)
        files = self.rec_files
        self.status_pub.publish(HdtnRecStatus(num_files, files))
    
    def add_rec_file(self, bundle_name, tar_location):
        self.rec_files.append(bundle_name)

    def __init__(self):
        # initialize list of recieved files
        self.rec_files = ["slatt", "fwa"]

        # initialize node
        self.hdtn = None
        #self.hdtn_init()

        # start node
        rospy.init_node('hdtn_rec')

        # start file transfer service
        self.output_data_service = rospy.Service('hdtn_transfer_recieved_file', SendFileTo, self.get_data)
        
        # start hdtn status publisher
        rate = rospy.Rate(REFRESH_RATE)    # set rate
        self.status_pub = rospy.Publisher('rec_status', HdtnRecStatus, queue_size=10)
        
        while not rospy.is_shutdown():
            self.hdtn_status()
            rate.sleep()
        

node = RecNode()