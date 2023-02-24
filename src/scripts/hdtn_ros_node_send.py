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
        file_utils.clear_dir(req.dest_dir)

        dest_host = req.dest_host
        dest_eid = req.dest_eid
        settings = {
            "global": {
                "eid": "ipn:1.1",
                "logging": True
            },
            "send": {
                "enabled": True,
                "dest_host": dest_host,
                "dest_eid": dest_eid,
                "send_dir": "send"
            },
            "receive": {
                "enabled": False,
            },
        }

        logging.basicConfig(level=logging.DEBUG)
        self.hdtn = HDTN(settings, hdtn_root="../HDTN")

    def hdtn_send_data(self, req):
        """
        moves contents of specified directory to hdtn send directory
        :param req: msg which contains directory to move contents of.
        :return: None
        """
        file_utils.clear_dir(self.send_dir)
        file_utils.copy_directory(req.src_dir, self.send_dir)

    def hdtn_status(self):
        """
        publishes status of HDTN node
        :return: None
        """
        # start hdtn status publisher
        self.status_pub = rospy.Publisher('hdtn_status', status, queue_size=10)
        while not rospy.is_shutdown():
            if self.hdtn is not None:
                mode_stats = self.hdtn.stats.get()
                status = mode_stats.get('current_status')
                current_item = mode_stats.get('current_item')
                total_number = mode_stats.get('total_number')
                self.status_pub.publish(status, current_item, total_number)
            else:
                self.status_pub.publish("HDTN node not initialized")  # TODO might be a more appropriate status message
            rospy.sleep(REFRESH_RATE)

    def __init__(self):
        self.hdtn = None

        # create receive directory
        self.send_dir = "" # TODO I'm not sure how to get the send directory from the settings
        file_utils.setup_directory(self.send_dir)

        # start file transfer service
        self.send_data_service = rospy.Service('hdtn_send_data', transfer_send, self.hdtn_send_data)

        # start hdtn destination service
        self.init_new_dst_service = rospy.Service('hdtn_dest', set_dest, self.hdtn_init)

        # start hdtn status publisher
        self.hdtn_status()
