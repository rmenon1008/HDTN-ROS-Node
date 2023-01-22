#!/usr/bin/env python

from __future__ import print_function

from services_def import SendData, SendDataResponse
import rospy

def handleDataRequest(req):
    print("Received data: " + req.data)
    return SendDataResponse()

def hdtn_service():
    rospy.init_node('hdtn_service')
    s = rospy.Service('hdtn_service', SendData, handleDataRequest)
    print("Ready to receive data.")
    rospy.spin()

if __name__ == "__main__":
    hdtn_service()