#!/usr/bin/env python
from __future__ import print_function
from services_def import SendData, SendDataResponse
import rospy
import file_utils
import logging

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
        logging.basicConfig(level=logging.INFO)

        # initialize HDTN
        self.hdtn = HDTN_Sender(settings, hdtn_root="../HDTN")

        # start hdtn
        self.hdtn.start()

    def hdtn_send_data(self, req):
        """
        moves contents of specified directory to hdtn send directory
        :param req: msg which contains directory to move contents of.
        :return: None
        """
        src = req.src_dir
        self.hdtn.send(src)

    def hdtn_status(self):
        """
        publishes status of HDTN node
        :return: None
        """
        if self.hdtn is not None:
            # TODO publish status
            self.status_pub.publish("test")

    def __init__(self):
        self.hdtn = None

        # start file transfer service
        self.send_data_service = rospy.Service('hdtn_send_data', transfer_send, self.hdtn_send_data)

        # start hdtn destination service
        self.init_new_dst_service = rospy.Service('hdtn_dest', set_dest, self.hdtn_init)

        while not rospy.is_shutdown():
            self.hdtn_status()
            rospy.sleep(REFRESH_RATE)