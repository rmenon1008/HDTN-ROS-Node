#!/usr/bin/env python
from __future__ import print_function
from services_def import MoveData
import rospy
import logging

REFRESH_RATE = 1

class SendNode:
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
        self.hdtn = HDTN_Receiver(settings, hdtn_root="../HDTN")
        self.hdtn.start()


    def get_data(self, req):
        """
        copies contents of receive directory to specified directory
        :param req: msg which contains dst directory
        :return: None
        """
        dst = req.dst_dir


    def hdtn_status(self):
        """
        publishes status of HDTN node
        :return: None
        """
        # start hdtn status publisher
        self.status_pub = rospy.Publisher('hdtn_status', status, queue_size=10)
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
        # initialize node
        self.hdtn = None
        self.hdtn_init()

        # start file transfer service
        self.output_data_service = rospy.Service('hdtn_output_data', transfer_rec, self.get_data)

        # start hdtn status publisher
        while not rospy.is_shutdown():
            self.hdtn_status()
            rospy.sleep(REFRESH_RATE)