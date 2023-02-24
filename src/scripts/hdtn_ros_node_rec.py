#!/usr/bin/env python
from __future__ import print_function
from services_def import MoveData
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
    def hdtn_init(self):
        """
        initializes the HDTN node in receive mode
        :return: None
        """


    def get_data(self, req):
        """
        copies contents of receive directory to specified directory
        :param req: msg which contains dst directory
        :return: None
        """
        file_utils.move_directory(req.src_dir)
        file_utils.clear_dir(req.dest_dir)


    def hdtn_status(self):
        """
        publishes status of HDTN node
        :return: None
        """
        # start hdtn status publisher
        self.status_pub = rospy.Publisher('hdtn_status', SendData, queue_size=10)
        while not rospy.is_shutdown():
            if self.hdtn is not None:
                mode_stats = self.hdtn.stats.get(mode)
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
        dest = ""
        file_utils.setup_directory(dest)

        # start file transfer service
        self.send_data_service = rospy.Service('hdtn_send_data', SendData, self.hdtn_send_data)

        # start hdtn destination service
        self.init_new_dst_service = rospy.Service('hdtn_dest', SendData, self.hdtn_init)

        # start hdtn status publisher
        self.hdtn_status()
