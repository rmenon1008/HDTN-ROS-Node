#!/usr/bin/env python
from __future__ import print_function
import rospy
import logging
from hdtn_fake import HDTN_Sender
from hdtn_node.srv import SendFileFrom, SendFileFromResponse, SetHdtnDest, SetHdtnDestResponse

REFRESH_RATE = 1

class SendNode:
    """
    This should initialize the HDTN with send config.

    Service: SendData: input is a file path, contents of receive is copied and moved to output path.
    Service: New Destination: input is a destination host and eid, HDTN is initialized with new destination.
    Publisher: Status of HDTN node
    """
    def hdtn_init(self, req):
        """
        initializes the HDTN node in send mode, erases the send directory first so no files are sent
        :param dest: destination node for HDTN
        :return: None
        """

        # get settings from request
        eid = req.eid
        dest_host = req.dest_host
        dest_eid = req.dest_eid
        settings = {
            "eid": eid,
            "dest_host": dest_host,
            "dest_eid": dest_eid,
        }

        # initialize HDTN
        self.hdtn = HDTN_Sender(settings, hdtn_root="../HDTN")

        # start hdtn
        self.hdtn.start()

        return SetHdtnDestResponse()

    def hdtn_send_data(self, req):
        """
        moves contents of specified directory to hdtn send directory
        :param req: msg which contains directory to move contents of.
        :return: None
        """
        if self.hdtn is not None:
            src = req.filepath
            self.hdtn.send_item(src)

            return SendFileFromResponse()
        raise RuntimeError("HDTN node must be initialized first")

    def hdtn_status(self): # not sure if we need this
        """
        publishes status of HDTN node
        :return: None
        """
        if self.hdtn is not None:
            # TODO publish status
            self.status_pub.publish("test")

    def __init__(self):
        self.hdtn = None

        # start node
        rospy.init_node('hdtn_rec')

        # start file transfer service
        self.send_data_service = rospy.Service('hdtn_send_file_from', SendFileFrom, self.hdtn_send_data)

        # start hdtn destination service
        self.init_new_dst_service = rospy.Service('set_hdtn_rec_dest', SetHdtnDest, self.hdtn_init)

        # start status pub
        # self.status_pub = rospy.Publisher('send_node_status', )
        rate = rospy.Rate(REFRESH_RATE)    # set rate

        while not rospy.is_shutdown():
            # self.hdtn_status()
            rate.sleep() 

send = SendNode()