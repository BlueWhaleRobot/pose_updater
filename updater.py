#!/usr/bin/env python
#encoding=utf-8
"""
获取位置信息，通过网络请求发送数据
"""

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading
import requests
import json
import uuid
import datetime
import re
from tf.listener import TransformListener
import time

SERVER_IP = "127.0.0.1"
SERVER_PORT = "9900"
API_URL = "createHeartBeatReport"

LASTEST_POSE = [0, 0]
POSE_LOCK = threading.RLock()
SHARPLINK_LOG_FILE= "/home/xiaoqiang/Documents/ros/devel/lib/sharplink/server.log"

TRANS_POSE = None

def get_my_id():
    log_file = open(SHARPLINK_LOG_FILE)
    contents = log_file.read()
    log_file.close()
    mid_search = re.search(r"[0-9]+,\sID:\s(?P<id>[0-9A-F]{76})", contents)
    mid = mid_search.group("id")
    return mid

def odom_cb(odom):
    global POSE_LOCK, LASTEST_POSE
    pose_stamped = PoseStamped()
    pose_stamped.header = odom.header
    pose_stamped.pose = odom.pose.pose
    trans_pose_stamped = None
    try:
        TRANS_POSE.waitForTransform(pose_stamped.header.frame_id, "/map",
            pose_stamped.header.stamp, rospy.Duration(0.1))
        trans_pose_stamped = TRANS_POSE.transformPose('/map', pose_stamped)
    except (tf.LookupException,
        tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
        rospy.logwarn("tf transform from /odom /map error")
        print(e)
    if trans_pose_stamped == None:
        return
    with POSE_LOCK:
        res = {}
        res["x"] = trans_pose_stamped.pose.position.x
        res["y"] = trans_pose_stamped.pose.position.y
        LASTEST_POSE = [int(1000 * res["x"]), int(1000 * res["y"]), ]


if __name__ == "__main__":
    rospy.init_node("pose_updater", anonymous=True)
    rate = rospy.Rate(0.5)
    TRANS_POSE = TransformListener()
    rospy.Subscriber("/xqserial_server/Odom", Odometry, odom_cb)
    while not rospy.is_shutdown():
        with POSE_LOCK:
            headers = {'content-type': 'application/json'}
            url = "http://{SERVER_IP}:{SERVER_PORT}/{API_URL}".format(
                SERVER_IP=SERVER_IP,
                SERVER_PORT=SERVER_PORT,
                API_URL=API_URL,
            )

            data = {
                "reqCode": str(uuid.uuid1()).replace("-", "")[:30].upper(),
                "reqTime": datetime.datetime.now().strftime("%G-%m-%e %H:%M:%S"),
                "wareHouseCode": "WMWHSE3",
                "sourceName": "GALILEO",
                "state": 0,
                "deviceNo": get_my_id()[:30],
                "currentX": LASTEST_POSE[0],
                "currentY": LASTEST_POSE[1],
                "currentSpeed": "0.0",
                "buildingFloor": "1",
                "batteryLevel": 100,
                "currentPlanDistance": 0,
                "currentFilishDistance": 0,
                "cmdId": "",
            }
            print(json.dumps(data, indent=4))
            # requests.post(url, data=json.dumps(data), headers=headers)
        rate.sleep()
